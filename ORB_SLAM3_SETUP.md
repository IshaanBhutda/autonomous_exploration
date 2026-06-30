# ORB-SLAM3 Monocular Visual SLAM Setup

This is **optional** and **separate** from the autonomous exploration. ORB-SLAM3 runs in parallel off the rover's camera feed (`/camera/image_raw`), building a sparse 3D visual feature map shown in a Pangolin window. It does not feed Nav2 — monocular SLAM has no metric scale — it's for visualizing the camera's trajectory and the room's 3D structure.

> **Heads-up:** ORB-SLAM3 is not a `pip`/`apt` install. It's a C++ library built from source plus a ROS 2 wrapper. The build is finicky (Pangolin, OpenCV version matching, Sophus). Budget 30–60 minutes. These steps are what worked on **Ubuntu 22.04.5, ROS 2 Humble, OpenCV 4.5.4, Eigen 3.4, GCC 11**.

## Stage 1 — Pangolin (the viewer)

```bash
# dependencies
sudo apt update
sudo apt install -y libgl1-mesa-dev libglew-dev libpython2.7-dev \
  libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
  libeigen3-dev libboost-all-dev libssl-dev

# build Pangolin v0.6 (newer versions break stock ORB-SLAM3)
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.6
```

**GCC 11 fix** — Pangolin v0.6's `colour.h` uses `std::numeric_limits` without including `<limits>`, which GCC 11 rejects. Add the include and disable the Python bindings (which trigger the error and aren't needed):

```bash
grep -q "#include <limits>" include/pangolin/gl/colour.h || \
  sed -i '0,/#include/s//#include <limits>\n#include/' include/pangolin/gl/colour.h

mkdir build && cd build
cmake .. -DBUILD_PANGOLIN_PYTHON=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

Verify: `ls /usr/local/lib/libpango*` should show `libpangolin.so`.

## Stage 2 — ORB-SLAM3 core library

Because the system has OpenCV 4.5.4 (stock ORB-SLAM3 targets 4.2), use the OpenCV-4.5-compatible fork:

```bash
cd ~
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This compiles the bundled Thirdparty libs (DBoW2, g2o, Sophus) then ORB-SLAM3. **10–20 minutes.**

> If `build.sh` dies with `c++: fatal error: Killed` (out of memory), edit `build.sh` and change every `make -j` to `make -j2`, then rerun.

Verify and extract the vocabulary:

```bash
ls ~/ORB_SLAM3/lib/libORB_SLAM3.so          # should exist (~5 MB)
cd ~/ORB_SLAM3/Vocabulary
ls ORBvoc.txt 2>/dev/null || tar -xf ORBvoc.txt.tar.gz
ls -la ORBvoc.txt                            # ~140 MB
```

Install Sophus system-wide (the ROS 2 wrapper needs it):

```bash
cd ~/ORB_SLAM3/Thirdparty/Sophus/build 2>/dev/null || \
  (mkdir -p ~/ORB_SLAM3/Thirdparty/Sophus/build && cd ~/ORB_SLAM3/Thirdparty/Sophus/build)
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
sudo make install
```

## Stage 3 — the ROS 2 wrapper

```bash
sudo apt install -y ros-humble-vision-opencv ros-humble-message-filters

cd ~/rover_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

The wrapper has hardcoded paths that must be fixed before it builds:

```bash
cd ~/rover_ws/src/orbslam3_ros2

# 1. PYTHONPATH: foxy -> humble
sed -i 's|/opt/ros/foxy/lib/python3.10/site-packages/|/opt/ros/humble/lib/python3.10/site-packages/|' CMakeLists.txt

# 2. ORB_SLAM3 root dir (replace <user> with your username)
sed -i 's|set(ORB_SLAM3_ROOT_DIR "/Home/ORB_SLAM3")|set(ORB_SLAM3_ROOT_DIR "/home/<user>/ORB_SLAM3")|' CMakeModules/FindORB_SLAM3.cmake
```

The stereo nodes fail to link (`opencv_calib3d` DSO missing) and aren't needed for monocular. Edit `CMakeLists.txt` to build only `mono` and `rgbd`, add explicit OpenCV linking, and use C++17. The relevant parts should read:

```cmake
# Default to C++17 (was 14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ...
find_package(OpenCV 4.2 REQUIRED)     # add this
find_package(ORB_SLAM3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${OpenCV_INCLUDE_DIRS}              # add this
)

# ---- Monocular node ----
add_executable(mono src/monocular/mono.cpp src/monocular/monocular-slam-node.cpp)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(mono ${OpenCV_LIBS})    # add this

# ---- RGB-D node ----
add_executable(rgbd src/rgbd/rgbd.cpp src/rgbd/rgbd-slam-node.cpp)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(rgbd ${OpenCV_LIBS})    # add this

# remove the add_executable(stereo ...) and add_executable(stereo-inertial ...) blocks
install(TARGETS mono rgbd DESTINATION lib/${PROJECT_NAME})
```

Build just the wrapper:

```bash
cd ~/rover_ws
colcon build --symlink-install --packages-select orbslam3
source install/setup.bash
```

## Camera config

Create a monocular config matching the simulated camera (640×480, computed intrinsics fx=fy≈381.39, cx=320, cy=240, zero distortion). Save it as `~/rover_ws/src/orbslam3_ros2/config/monocular/gazebo_mono.yaml`. Base it on the wrapper's `TUM1.yaml` and replace the camera intrinsics block. (If your camera resolution/FOV differs, recompute fx = width / (2·tan(hfov/2)).)

## Running it

ORB-SLAM3's runtime library path must be on `LD_LIBRARY_PATH`. Add it permanently:

```bash
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/ORB_SLAM3/lib:$HOME/ORB_SLAM3/Thirdparty/DBoW2/lib:$HOME/ORB_SLAM3/Thirdparty/g2o/lib' >> ~/.bashrc
source ~/.bashrc
```

Then, with the sim running (e.g. `slam.launch.py` or `explore.launch.py` in another terminal):

```bash
ros2 run orbslam3 mono \
  ~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/rover_ws/src/orbslam3_ros2/config/monocular/gazebo_mono.yaml \
  --ros-args -r camera:=/camera/image_raw
```

The `-r camera:=/camera/image_raw` remap is essential — it points the node at the rover's camera topic. A Pangolin window opens showing feature tracking.

> **Monocular initialization needs translation, not rotation.** Drive the rover *forward* a bit to bootstrap tracking — pure spinning won't initialize a monocular map.

## Running alongside autonomous exploration

ORB-SLAM3 only reads the camera; it doesn't touch `/cmd_vel` or `/map`. So you can run it at the same time as `explore.launch.py` — you'll get the 2D lidar occupancy map (driving the autonomy) and the ORB-SLAM3 3D visual map building simultaneously.

> Caveat: ORB-SLAM3 + Nav2 + SLAM Toolbox together are CPU-heavy. If the sim drops below real-time, ORB tracking can get rough (it's sensitive to frame timing). If tracking is unstable, run ORB-SLAM3 separately from the full autonomy stack.

## Troubleshooting

**`libORB_SLAM3.so: cannot open shared object file`** — `LD_LIBRARY_PATH` not set in that terminal. Run the export above. Verify with:
```bash
ldd ~/rover_ws/install/orbslam3/lib/orbslam3/mono | grep "not found"
```
Empty output = all libraries resolve.

**Pangolin build errors on `numeric_limits`** — the `<limits>` include fix in Stage 1 wasn't applied; re-run it.

**Wrapper link error on `opencv_calib3d`** — the stereo nodes weren't removed; only `mono`/`rgbd` should be in `add_executable`/`install`.

## Credits

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (UZ-SLAMLab) — the core algorithm.
- [zang09/ORB-SLAM3-STEREO-FIXED](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED) — OpenCV 4.5-compatible fork.
- [zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2) — the ROS 2 wrapper.
- [Pangolin](https://github.com/stevenlovegrove/Pangolin) — the viewer.
