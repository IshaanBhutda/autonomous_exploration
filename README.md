# autonomous_exploration

Autonomous frontier-based exploration for a differential-drive rover in ROS 2 Humble + Gazebo Classic, with optional ORB-SLAM3 monocular visual SLAM running alongside.

The rover drives itself around an unknown indoor environment, builds a 2D occupancy map with lidar SLAM, and decides on its own where to explore next using frontier detection — until the whole space is mapped. No waypoints, no manual driving.

![status](https://img.shields.io/badge/ROS_2-Humble-blue) ![status](https://img.shields.io/badge/Gazebo-Classic_11-orange) ![license](https://img.shields.io/badge/license-MIT-green)

---

## What it does

- **Spawns** a 4-wheel skid-steer rover (2D lidar + RGB camera) in a Gazebo world.
- **Maps** the environment live with [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) (lidar → occupancy grid).
- **Navigates** with [Nav2](https://navigation.ros.org/) (NavFn global planner + DWB local controller).
- **Explores autonomously** via a custom frontier-exploration node: it finds the boundary between known and unknown space, picks the best frontier, sends it to Nav2 as a goal, and repeats until nothing unknown remains.
- **(Optional) Visual SLAM** with ORB-SLAM3 monocular, running in parallel off the same camera feed — see [`ORB_SLAM3_SETUP.md`](ORB_SLAM3_SETUP.md).

## How the autonomy works

Two separate "brains" cooperate:

| Component | Decides | Role |
|-----------|---------|------|
| **Explorer** (this package) | **WHERE** to go | Finds frontiers (free cells bordering unknown space), scores them by size and distance, sends the best as a goal. |
| **Nav2** (library) | **HOW** to get there | Plans a path around walls and drives the wheels, avoiding obstacles. |
| **SLAM Toolbox** (library) | the map + where the robot is | Builds `/map` and the `map → base_footprint` transform. |

The loop, every cycle:

1. Read the latest `/map`.
2. Find **frontiers** — free cells next to unknown cells (the edge of the explored region).
3. **Cluster** connected frontier cells into groups (BFS flood fill); discard tiny ones.
4. **Score** each cluster by size (bigger = more to discover) minus distance (closer = cheaper).
5. Send the best frontier's centroid to Nav2 as a `NavigateToPose` goal.
6. On arrival, re-evaluate. On failure, blacklist the spot, back out, try another.

When no frontiers remain, the map is complete and the rover stops.

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full data-flow and a line-by-line walk through the explorer algorithm.

---

## 🎥 Demo Video

[![Autonomous Exploration Demo](assets/demo_thumbnail.png)](https://github.com/IshaanBhutda/autonomous_exploration/blob/main/assets/demo.mp4)

## Quick start

> Requires Ubuntu 22.04, ROS 2 Humble, and Gazebo Classic 11. Full dependency list in [`INSTALL.md`](INSTALL.md).

```bash
# 1. clone into a workspace
mkdir -p ~/rover_ws/src && cd ~/rover_ws/src
git clone https://github.com/IshaanBhutda/autonomous_exploration.git

# 2. install dependencies
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. build
colcon build --packages-select autonomous_exploration
source install/setup.bash

# 4. run autonomous exploration
ros2 launch autonomous_exploration explore.launch.py
```

Give it ~16 seconds. The rover spins to bootstrap SLAM, then explores on its own. Watch the map, costmaps, frontier markers, and planned path build live in RViz.

### Other launch files

| Launch | What it does |
|--------|--------------|
| `explore.launch.py` | **Full autonomy**: rover + SLAM + Nav2 + explorer. The main one. |
| `slam.launch.py` | Rover + SLAM only — drive manually to build a map. |
| `spawn_only.launch.py` | Just the rover in Gazebo (no SLAM/Nav2). For testing the model. |
| `nav2.launch.py` | Nav2 bringup only (included by `explore.launch.py`). |

### Driving manually (for `slam.launch.py`)

Teleop sends per-keypress; for continuous motion publish directly:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 20
```

Save a map when done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

---

## Use your own world

The default world is a multi-room building. To test the explorer in **your** environment, see [`CUSTOM_WORLD.md`](CUSTOM_WORLD.md) — it covers dropping in a `.world` file, the one required Gazebo plugin for `/clock`, and tuning the navigation for your corridor widths.

Short version:

1. Put your `.world` in `worlds/`.
2. Make sure it has the `libgazebo_ros_init` + `libgazebo_ros_factory` plugins (so `/clock` publishes).
3. Point `spawn_only.launch.py` at it.
4. If your corridors are narrow, tune `robot_radius` / `inflation_radius` in `config/nav2_params.yaml` (see the guide).

---

## Tuning

The two config files you'll touch most:

**`config/explore_params.yaml`** — exploration behavior:
- `min_frontier_size` — ignore frontiers smaller than this (raise it to skip tight pockets).
- `blacklist_radius` / `blacklist_clear_period` — how failed goals are excluded and retried.
- `min_goal_distance` — reject frontiers closer than this.

**`config/nav2_params.yaml`** — navigation:
- `robot_radius` (in both costmaps) — footprint size for planning.
- `inflation_radius` / `cost_scaling_factor` — keep-out distance from walls. Critical for narrow spaces (see `CUSTOM_WORLD.md`).
- DWB critic weights — corner/path-following behavior.

---

## Repository layout

```
autonomous_exploration/
├── config/
│   ├── explore_params.yaml   # frontier exploration parameters
│   ├── nav2_params.yaml      # Nav2 (DWB) configuration
│   └── slam_toolbox.yaml     # SLAM Toolbox configuration
├── launch/
│   ├── explore.launch.py     # full autonomy (main entry point)
│   ├── nav2.launch.py        # Nav2 bringup
│   ├── slam.launch.py        # SLAM only (manual driving)
│   └── spawn_only.launch.py  # rover in Gazebo only
├── autonomous_exploration/
│   └── explore_node.py     # the frontier exploration node
├── rviz/                     # RViz configs (nav, slam, spawn)
├── urdf/
│   └── rover.urdf.xacro      # the rover model
├── worlds/
│   └── custom_world.world    # default test environment
├── package.xml
└── setup.py
```

---

## Documentation

- [`INSTALL.md`](INSTALL.md) — full dependency install for a fresh machine.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — system design + the explorer algorithm explained.
- [`CUSTOM_WORLD.md`](CUSTOM_WORLD.md) — drop in your own world and tune for it.
- [`ORB_SLAM3_SETUP.md`](ORB_SLAM3_SETUP.md) — build and run ORB-SLAM3 visual SLAM alongside.

## Acknowledgements

Built on [ROS 2](https://ros.org/), [Nav2](https://navigation.ros.org/), [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox), [Gazebo](https://classic.gazebosim.org/), and [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). The frontier exploration node is original work in this repository.

## License

MIT — see [`LICENSE`](LICENSE).
