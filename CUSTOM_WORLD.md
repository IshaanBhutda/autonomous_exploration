# Using Your Own World

This guide shows how to drop in a custom Gazebo world and test the exploration algorithm in it.

## 1. Add your world file

Put your `.world` file in the `worlds/` directory:

```
autonomous_exploration/worlds/my_world.world
```

## 2. Add the required Gazebo plugins (important)

Your world **must** publish simulated time (`/clock`) and allow models to be spawned, or SLAM and Nav2 won't work (they run on `use_sim_time: true`). Add these two plugins inside the `<world>` tag, right after the opening `<world name='...'>` line:

```xml
<world name='default'>

  <plugin name='gazebo_ros_init' filename='libgazebo_ros_init.so'/>
  <plugin name='gazebo_ros_factory' filename='libgazebo_ros_factory.so'/>

  <!-- ... rest of your world ... -->
</world>
```

> This is the single most common reason a custom world "does nothing" — without the init plugin, `/clock` never publishes, every node waits forever for time, and nothing moves. The bundled `custom_world.world` already has these.

## 3. Point the launch at your world

Edit `launch/spawn_only.launch.py` and change the world path to your file. Look for where the world is passed to Gazebo (the `gzserver` / `world` argument) and set it to `my_world.world`. Rebuild:

```bash
cd ~/rover_ws
colcon build --packages-select autonomous_exploration
source install/setup.bash
```

## 4. Set a good spawn point

The rover spawns at a fixed location (default origin). If your world has walls there, the rover spawns inside a wall. In `spawn_only.launch.py`, set the spawn `-x`, `-y`, `-z` to an open spot in your world. Keep `-z` slightly above the floor (e.g. `0.15`) so the rover drops onto its wheels.

## 5. Tune navigation for your environment

This is where most of the real work is. The default tuning assumes corridors roughly 0.8 m or wider. If your world has different geometry, adjust `config/nav2_params.yaml`.

### The key constraint: footprint + inflation vs corridor width

The rover keeps a "keep-out" distance from every wall equal to `robot_radius + inflation_radius`. If that exceeds half your corridor width, **there's no valid path through and the rover gets stuck** — not because it physically can't fit, but because the costmap marks the whole corridor as high-cost.

```
keep-out = robot_radius + inflation_radius
```

For a corridor of width `W`, you need:

```
robot_radius + inflation_radius  <  W / 2
```

The defaults in this repo:

| Parameter | Default | Where |
|-----------|---------|-------|
| `robot_radius` | 0.18 | both `local_costmap` and `global_costmap` |
| `inflation_radius` | 0.18 | both inflation layers |
| `cost_scaling_factor` | 3.0 | both inflation layers |

So default keep-out = 0.36 m → corridors **wider than ~0.75 m** work cleanly.

### If your corridors are narrower

- Lower `inflation_radius` (e.g. to 0.12). The rover hugs walls more but fits tighter spaces.
- Lower `robot_radius` — but **not below the rover's true half-width** (0.11 m for the default 0.22 m-wide chassis), or it will clip walls.
- Raise `cost_scaling_factor` so cost drops off faster from walls, leaving a clearer centerline.

### If your corridors are wide and it still hugs corners

- Raise `BaseObstacle.scale` (default 0.5) so obstacle clearance competes with path-following.
- Lower `PathDist.scale` (default ~24) so the rover is freer to deviate from the corner-cutting global path.

## 6. Tune exploration behavior

In `config/explore_params.yaml`:

- **`min_frontier_size`** (default 12) — raise it to make the rover ignore small openings and tight pockets (avoids getting trapped in alcoves). Lower it to explore more thoroughly into small spaces.
- **`min_goal_distance`** (default 0.3) — frontiers closer than this are rejected. Raise it if the rover fixates on nearby spots.
- **`blacklist_radius`** (default 0.7) — when a goal fails, how big an area gets temporarily excluded.
- **`blacklist_clear_period`** (default 30) — how often the blacklist resets so previously-failed regions are retried.

## 7. Run

```bash
pkill -9 -f gz; pkill -9 -f ros2; rm -rf /dev/shm/fastrtps_*; sleep 3
ros2 launch autonomous_exploration explore.launch.py
```

## Building a world from scratch

If you want to design a world rather than import one, the easiest path is the Gazebo Building Editor:

1. Launch Gazebo: `gazebo`
2. **Edit → Building Editor** (`Ctrl+B`).
3. Draw walls, add doors/windows, set wall heights.
4. **File → Save**, then exit the editor.
5. Add furniture/obstacles from the model database if you like.
6. **File → Save World As** → save into `worlds/`.
7. Open the saved `.world` in a text editor and add the two Gazebo plugins from step 2.

Keep corridors **at least 0.8 m wide** for the default tuning, or plan to adjust the navigation parameters as above.

## Common pitfalls

- **Rover spawns inside a wall** → fix the spawn coordinates (step 4).
- **Everything frozen, no movement** → missing `/clock` plugin (step 2).
- **Rover gets stuck in corridors** → keep-out exceeds corridor half-width (step 5).
- **Rover drives into dead-end alcoves** → raise `min_frontier_size` (step 6).
- **SLAM map looks smeared/doubled** → drive slower, or your world has too few features for scan-matching; add some structure.
