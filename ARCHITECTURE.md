# Architecture

How the system fits together, and how the frontier exploration algorithm works.

## System overview

```
                        ┌──────────────┐
                        │    Gazebo    │  simulates the rover + world,
                        │   (Classic)  │  publishes sensor data + /clock
                        └──────┬───────┘
                  /scan        │       /camera/image_raw
            ┌──────────────────┼───────────────────┐
            │                  │                   │
            ▼                  ▼                   ▼
     ┌─────────────┐    drive plugin        ┌──────────────┐
     │ SLAM Toolbox│    listens /cmd_vel    │  ORB-SLAM3   │ (optional,
     │             │    publishes /odom     │  monocular   │  parallel)
     │  builds     │    + odom→base TF      └──────────────┘
     │  /map +     │
     │  map→odom TF│
     └──────┬──────┘
            │ /map
            ▼
     ┌─────────────────┐  picks WHERE to go
     │  autonomous_exploration │  (frontier detection + scoring)
     │   (this pkg)    │
     └──────┬──────────┘
            │ NavigateToPose goal
            ▼
     ┌─────────────────┐  figures out HOW to get there
     │      Nav2       │  (NavFn planner + DWB controller)
     │                 │  publishes /cmd_vel → drive plugin
     └─────────────────┘
```

## The transform tree

```
map ──(SLAM Toolbox)──> odom ──(drive plugin)──> base_footprint ──> base_link ──> wheels / lidar / camera
```

- **map → odom**: published by SLAM Toolbox from scan-matching. Corrects odometry drift.
- **odom → base_footprint**: published by the Gazebo diff-drive plugin from wheel odometry.
- **base_footprint → sensors**: static, from the URDF.

The explorer needs `map → base_footprint` to know where the robot is on the map. SLAM provides the first half; the drive plugin provides the second.

## Division of labor

The key design idea: **separate WHERE from HOW.**

- **The explorer decides WHERE.** It looks at the map and picks a destination based purely on the explore-the-unknown logic. It has no idea how to drive.
- **Nav2 decides HOW.** Given a destination, it plans a collision-free path and turns it into wheel velocities. It has no idea *why* it's going there.

Neither is "autonomous" alone — the explorer would pick goals it can't reach, and Nav2 would sit idle with no goal. Together they produce autonomous exploration.

---

## The frontier exploration algorithm

The explorer node (`autonomous_exploration/explore_node.py`) runs a loop every `replan_period` seconds. Here is each stage.

### 1. Read the map

Subscribes to `/map` (an `OccupancyGrid`). Each cell is one of:
- `0` — **free** (lidar has seen through it)
- `100` — **occupied** (wall/obstacle)
- `-1` — **unknown** (never observed)

### 2. Find frontiers

A **frontier cell** is a free cell that has at least one unknown neighbor — i.e. it sits on the boundary between explored and unexplored space. Driving to a frontier reveals new territory.

This is done with vectorized NumPy (no per-cell loop). The map is turned into two boolean masks (`free`, `unknown`), and a shifted-OR trick checks each cell's four neighbors for unknown space at once:

```python
nbr_unknown[1:, :]  |= unknown[:-1, :]   # is the cell above unknown?
nbr_unknown[:-1, :] |= unknown[1:, :]    # below?
nbr_unknown[:, 1:]  |= unknown[:, :-1]   # left?
nbr_unknown[:, :-1] |= unknown[:, 1:]    # right?
return free & nbr_unknown                # frontier = free AND neighbor-unknown
```

### 3. Cluster frontiers

Individual frontier cells get grouped into connected blobs so one doorway is one target, not 200. This is **connected-component labeling via BFS flood fill**:

```python
for each unvisited frontier cell:
    start a new cluster, seed a queue with this cell
    while queue not empty:
        pop a cell, add it to the cluster
        for each of its 8 neighbors:
            if neighbor is also a frontier cell and unvisited:
                mark visited, add to queue
    keep the cluster only if it has >= min_frontier_size cells
```

The `min_frontier_size` filter discards noise and tiny pockets. Each surviving cluster is reduced to its **centroid** — the average position of its cells — which becomes the candidate goal. The cluster's size feeds the scoring.

### 4. Score and select

Each cluster's centroid is scored:

```
score = size_weight · log(cluster_size + 1)  −  distance_weight · distance_to_robot
```

Bigger frontiers (more unknown space behind them) score higher; farther ones score lower. Clusters that are too close (`min_goal_distance`) or blacklisted are skipped. The highest-scoring frontier wins.

### 5. Send the goal

The winning centroid becomes a `NavigateToPose` action goal, with orientation pointing from the robot toward the frontier. Nav2 takes over the driving.

### 6. Handle the result

- **Succeeded** → loop back to step 1 with a now-larger map.
- **Failed/aborted** (rover stuck) → blacklist the frontier, reverse briefly to escape the pocket (`escape_backup`), then try a different frontier next cycle.

The blacklist clears every `blacklist_clear_period` seconds so a transiently-failed region isn't excluded forever.

### Self-start (the bootstrap)

At startup the map is just a small disc around the rover, so every frontier is within `min_goal_distance` and gets rejected — the explorer would sit still. To break this chicken-and-egg, when it has a map but no usable frontier *and* hasn't explored yet, it **spins in place** (`do_startup_spin`), publishing rotation to `/cmd_vel`. The lidar sweeps a full circle, the map grows, distant frontiers appear, and exploration begins — no manual nudge needed.

### Termination

When no frontiers remain, every reachable free cell is surrounded by known space — the environment is fully mapped. The explorer logs completion and stops sending goals.

---

## Parameter reference

### `explore_params.yaml`

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `min_frontier_size` | 12 | Min cells for a frontier cluster to count. |
| `distance_weight` | 2.0 | How much closer frontiers are preferred. |
| `size_weight` | 1.0 | How much bigger frontiers are preferred. |
| `replan_period` | 1.0 | Seconds between exploration cycles. |
| `blacklist_radius` | 0.7 | Exclusion radius around a failed goal (m). |
| `min_goal_distance` | 0.3 | Reject frontiers closer than this (m). |
| `blacklist_clear_period` | 30.0 | Seconds before the blacklist resets. |

### `nav2_params.yaml` (most-tuned)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `robot_radius` | 0.18 | Circular footprint for planning. |
| `inflation_radius` | 0.18 | Wall keep-out gradient distance. |
| `cost_scaling_factor` | 3.0 | How fast inflation cost falls off. |
| `BaseObstacle.scale` | 0.5 | DWB obstacle-avoidance weight. |
| `PathDist.scale` | ~24 | DWB path-following weight. |

See [`CUSTOM_WORLD.md`](CUSTOM_WORLD.md) for how these interact with corridor width.
