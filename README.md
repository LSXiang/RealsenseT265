# Realsense-T265

Since the Realsense T265 has been officially removed from support, in order to avoid multiple version conflicts, this repository backtracks to the official libraries and ROS2 packages that can support the T265 model, and builds them from source code for local installation and operation.

## Installation Instructions
The repository is compiled and tested in the following system environment, and other environments are modified according to the actual situation.

### Step 1: Install the ROS2 distribution
- **Ubuntu 22.04:**
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Step 2: Download source
  ```bash
  cd [ros2_ws]/src
  git clone http://192.168.20.16/SLAM-Group/RealsenseT265.git
  ```

### Step 3: Build and install the Intel&reg; RealSense&trade; SDK 2.0
  ```bash
  cd [path_librealsense] && mkdir build && cd build
  cmake -DCMAKE_INSTALL_PREFIX=../install ..
  make & make install
  ```

### Step 4: Install dependencies
  ```bash
  cd [ros2_ws]/src
  sudo apt-get install python3-rosdep -y
  sudo rosdep init # "sudo rosdep init --include-eol-distros" for Dashing
  rosdep update
  rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
  ```
### Step 5: Build
  ```bash
  colcon build --symlink-install --packages-ignore librealsense2 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
  ```

### Step 6: Terminal environment
  ```bash
  ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: humble, galactic, foxy, eloquent, dashing
  source /opt/ros/$ROS_DISTRO/setup.bash
  cd [ros2_ws]
  source install/local_setup.bash
  ```

## Usage Instructions

### Start the camera node
To start the camera node:

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_pose:=true -p device_type:=t265
```
or, with a launch file:
```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py enable_pose:=true device_type:=t265
```

### Start the T265 with a launch and configuration file (recommend)
```bash
ros2 launch realsense2_camera rs_t265_launch.py
```

## See [librealsense/readme](./librealsense/readme.md) & [realsense-ros/readme](./realsense-ros/README.md) for more details

