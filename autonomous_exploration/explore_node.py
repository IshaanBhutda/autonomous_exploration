#!/usr/bin/env python3
"""
Autonomous frontier-based exploration.

Reads /map from SLAM Toolbox, finds frontiers (free cells bordering unknown
space), clusters and scores them, sends the best as a NavigateToPose goal to
Nav2. Repeat until no frontiers remain. Blacklist clears periodically so
transient Nav2 failures don't permanently exclude regions.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import tf2_ros

FREE = 0
UNKNOWN = -1


class RoverExplorer(Node):
    def __init__(self):
        super().__init__('explorer')

        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_frontier_size', 6)
        self.declare_parameter('distance_weight', 2.0)
        self.declare_parameter('size_weight', 1.0)
        self.declare_parameter('replan_period', 2.0)
        self.declare_parameter('blacklist_radius', 0.4)
        self.declare_parameter('min_goal_distance', 0.3)
        self.declare_parameter('blacklist_clear_period', 30.0)

        self.base_frame = self.get_parameter('robot_base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.distance_weight = self.get_parameter('distance_weight').value
        self.size_weight = self.get_parameter('size_weight').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.blacklist_clear_period = self.get_parameter('blacklist_clear_period').value

        self.map_msg = None
        self.blacklist = []
        self.current_goal = None
        self.target_frontier = None
        self.goal_active = False
        self.last_blacklist_clear = self.now_sec()
        self.has_explored = False  # becomes True after first successful goal
        self.spin_start = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 1)
        # Publisher for an initial in-place spin to grow the map at startup,
        # so distant frontiers appear and exploration can begin on its own.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        period = self.get_parameter('replan_period').value
        self.timer = self.create_timer(period, self.explore_step)
        self.get_logger().info('Rover explorer up. Waiting for /map and Nav2...')


    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def map_cb(self, msg):
        self.map_msg = msg

    def get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None

    def grid_to_world(self, mx, my, info):
        x = info.origin.position.x + (mx + 0.5) * info.resolution
        y = info.origin.position.y + (my + 0.5) * info.resolution
        return x, y

    def find_frontiers(self, grid):
        free = (grid == FREE)
        unknown = (grid == UNKNOWN)
        nbr_unknown = np.zeros_like(unknown)
        nbr_unknown[1:, :]  |= unknown[:-1, :]
        nbr_unknown[:-1, :] |= unknown[1:, :]
        nbr_unknown[:, 1:]  |= unknown[:, :-1]
        nbr_unknown[:, :-1] |= unknown[:, 1:]
        return free & nbr_unknown

    def cluster_frontiers(self, mask, w, h):
        visited = np.zeros_like(mask, dtype=bool)
        clusters = []
        ys, xs = np.where(mask)
        for (sy, sx) in zip(ys.tolist(), xs.tolist()):
            if visited[sy, sx]:
                continue
            q = deque([(sy, sx)])
            visited[sy, sx] = True
            comp = []
            while q:
                cy, cx = q.popleft()
                comp.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w and not visited[ny, nx] and mask[ny, nx]:
                            visited[ny, nx] = True
                            q.append((ny, nx))
            if len(comp) >= self.min_frontier_size:
                clusters.append(comp)
        return clusters

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius
                   for bx, by in self.blacklist)

    def explore_step(self):
        if self.map_msg is None or self.goal_active:
            return

        if self.now_sec() - self.last_blacklist_clear > self.blacklist_clear_period:
            if self.blacklist:
                self.get_logger().info('Clearing blacklist to retry regions.')
            self.blacklist = []
            self.last_blacklist_clear = self.now_sec()

        info = self.map_msg.info
        w, h = info.width, info.height
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(h, w)

        robot = self.get_robot_xy()
        if robot is None:
            return
        rx, ry = robot

        mask = self.find_frontiers(grid)
        clusters = self.cluster_frontiers(mask, w, h)
        if not clusters:
            self.get_logger().info('No frontiers left -> exploration COMPLETE.')
            self.publish_markers([], None)
            return

        best, best_score = None, -float('inf')
        candidates = []
        for comp in clusters:
            arr = np.array(comp)
            cy, cx = arr[:, 0].mean(), arr[:, 1].mean()
            wx, wy = self.grid_to_world(cx, cy, info)
            dist = math.hypot(wx - rx, wy - ry)
            if dist < self.min_goal_distance or self.is_blacklisted(wx, wy):
                continue
            score = self.size_weight * math.log(len(comp) + 1) - self.distance_weight * dist
            candidates.append((wx, wy, len(comp)))
            if score > best_score:
                best_score, best = score, (wx, wy)

        self.publish_markers(candidates, best)
        if best is None:
            # No usable frontier yet. If we've never successfully explored,
            # the map is probably just a tiny startup patch -> spin in place to
            # grow it so distant frontiers appear. This self-starts exploration
            # without a manual nudge.
            if not self.has_explored:
                self.do_startup_spin()
            else:
                self.get_logger().info('All frontiers blacklisted/too close; will retry after clear.')
            return

        # We found a frontier -> mark that exploration has begun (stop spinning).
        self.has_explored = True

        fx, fy = best
        self.target_frontier = (fx, fy)
        self.send_goal(fx, fy, rx, ry)

    def do_startup_spin(self):
        """Spin in place to grow the initial map until distant frontiers appear.
        Publishes directly to /cmd_vel. Runs only before the first real goal,
        when Nav2 has no active goal, so it isn't contested."""
        t = Twist()
        t.angular.z = 0.5
        self.cmd_pub.publish(t)
        self.get_logger().info('Startup spin: growing map to find frontiers...',
                               throttle_duration_sec=2.0)

    def send_goal(self, x, y, rx, ry):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 navigate_to_pose not available yet.')
            return
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        yaw = math.atan2(y - ry, x - rx)
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = ps

        self.current_goal = (x, y)
        self.goal_active = True
        self.get_logger().info(f'Exploring frontier at ({x:.2f}, {y:.2f})')
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.blacklist_current()
            self.goal_active = False
            return
        handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Reached frontier. Re-evaluating.')
        else:
            # Goal failed - the rover likely got stuck (e.g. drove into a tight
            # pocket). Blacklist this frontier AND back out a little so it can
            # escape and reach a different, more open frontier next cycle.
            self.get_logger().warn(f'Goal status {status}; blacklisting and backing out.')
            self.blacklist_current()
            self.escape_backup()
        self.goal_active = False

    def escape_backup(self):
        """Reverse briefly to escape a pocket/dead-end after a failed goal.
        Uses a short timed loop so the reverse actually lasts ~1.5s."""
        import time
        t = Twist()
        t.linear.x = -0.15
        end = self.now_sec() + 1.5
        while self.now_sec() < end:
            self.cmd_pub.publish(t)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())  # stop
        self.get_logger().info('Backed out; will try a different frontier.')

    def blacklist_current(self):
        # Blacklist the true frontier we were heading toward, not the
        # intermediate hop waypoint.
        target = self.target_frontier if self.target_frontier is not None else self.current_goal
        if target is not None:
            self.blacklist.append(target)

    def publish_markers(self, candidates, best):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, (x, y, size) in enumerate(candidates):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=x, y=y, z=0.1)
            m.pose.orientation.w = 1.0
            s = min(0.15 + 0.01 * size, 0.6)
            m.scale.x = m.scale.y = m.scale.z = s
            is_best = best is not None and abs(x - best[0]) < 1e-6 and abs(y - best[1]) < 1e-6
            m.color.r = 0.0 if is_best else 1.0
            m.color.g = 1.0 if is_best else 0.6
            m.color.b = 0.0
            m.color.a = 0.9
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = RoverExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
