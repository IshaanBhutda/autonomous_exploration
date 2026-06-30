# Installation

Full setup for a fresh machine. Tested on **Ubuntu 22.04.5 LTS**, **ROS 2 Humble**, **Gazebo Classic 11**.

## 1. ROS 2 Humble

If you don't have it, follow the [official ROS 2 Humble install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Install the desktop variant:

```bash
sudo apt install ros-humble-desktop
```

Source it (and add to your shell startup):

```bash
source /opt/ros/humble/setup.bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

## 2. Gazebo Classic + ROS integration

```bash
sudo apt install gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control
```

> This project uses **Gazebo Classic 11**, not Ignition/Gazebo Sim. The `gazebo_ros` plugins in the URDF are Classic plugins.

## 3. Navigation + SLAM packages

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2
```

## 4. Python dependencies

The explorer node uses NumPy (usually already present):

```bash
pip3 install numpy
```

## 5. Build the package

```bash
mkdir -p ~/rover_ws/src && cd ~/rover_ws/src
git clone https://github.com/<your-username>/autonomous_exploration.git

cd ~/rover_ws
# resolve any remaining deps automatically
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select autonomous_exploration
source install/setup.bash
```

Add the workspace to your shell startup so every terminal sees it:

```bash
echo 'source ~/rover_ws/install/setup.bash' >> ~/.bashrc
```

## 6. Verify

```bash
ros2 launch autonomous_exploration spawn_only.launch.py
```

You should see Gazebo open with the rover. If that works, run the full thing:

```bash
ros2 launch autonomous_exploration explore.launch.py
```

---

## Troubleshooting

**Gazebo opens but the rover doesn't move / no `/clock`.**
The world file must include the Gazebo init + factory plugins so simulated time publishes. See [`CUSTOM_WORLD.md`](CUSTOM_WORLD.md). The bundled `custom_world.world` already has them.

**Nav2 nodes stay `unconfigured` / no costmaps appear.**
Almost always a stale-shared-memory or timing issue. Clean up before launching:

```bash
pkill -9 -f gz; pkill -9 -f ros2
rm -rf /dev/shm/fastrtps_*
sleep 3
```

Then relaunch. You can verify Nav2 came up with:

```bash
ros2 lifecycle get /controller_server   # want: active [3]
```

**`controller_server` reports "Node not found" / crashes.**
This happens if `nav2_params.yaml` references a controller plugin that isn't installed (e.g. a rotation-shim controller). This repo's config uses only the always-installed `dwb_core::DWBLocalPlanner`, so a stock Nav2 install is sufficient.

**Rover won't auto-start exploring.**
At startup the map is tiny and all frontiers are too close to accept, so the explorer spins in place first to grow the map. Give it ~16 seconds. If it never starts, check the explorer terminal for `TF lookup failed` (SLAM not publishing `map → base_footprint` yet).

**ORB-SLAM3:** see [`ORB_SLAM3_SETUP.md`](ORB_SLAM3_SETUP.md) — it's a separate from-source build with its own dependencies.
