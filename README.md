# PicoXR ROS Integration Toolkit

## Project Overview

This repository provides a cross-platform (ROS1/ROS2) interface and toolkit for integrating PicoXR devices with robotics applications, especially for controlling dual-arm robots (such as ARX and UR5e) using XR (Extended Reality) input. It includes:

- **ROS1 and ROS2 packages** for interfacing with PicoXR devices and publishing XR pose and controller data.
- **Custom message definitions** for XR data (`xr_msgs`).
- **Example applications** for mapping XR input to robot control, including kinematics and visualization.
- **Assets** for robot models (URDF, MJCF, meshes) and simulation.

---

## Repository Structure

- `ros2/` and `ros1/`: ROS2 and ROS1 packages, each containing:
  - `picoxr`: Main interface node for PicoXR device data.
  - `xr_msgs`: Custom message definitions for XR pose and controller data.
- `examples/`:
  - `arx_ros2` and `arx_ros1`: Example applications for mapping XR input to ARX robots (with kinematics, control, and visualization).
  - `assets`: Robot models and simulation files (URDF, MJCF, meshes, etc.).
  - `python`: Python scripts for kinematics and simulation (e.g., Mujoco/URDF conversion, IK solvers).

---

## Main Features

### XR Data Interface

- **picoxr node** (ROS1/ROS2): Connects to PicoXR devices, parses device state JSON, and publishes XR pose and controller data as custom ROS messages.
- **xr_msgs**: Defines messages for XR head and controller state:
  - `Custom.msg`: Timestamp, input, head pose, left/right controller.
  - `Head.msg`: 7-DOF pose, status.
  - `Controller.msg`: Axes, buttons, gripper, trigger, 7-DOF pose, status.

### Example Applications

- **arx_ros2/arx_ros1**: 
  - Subscribe to XR pose data.
  - Convert XR pose to robot end-effector pose (ARX).
  - Use kinematics solvers (Placo, Eigen) to compute joint positions.
  - Publish joint commands to robot or simulation.
  - Visualization tools for URDF/MJCF robots.

- **assets**: 
  - Ready-to-use robot models for ARX and UR5e (URDF, MJCF, meshes).
  - Conversion scripts between URDF and MJCF.
  - Example scenes for simulation.

---

## Installation

### Dependencies

- ROS2 (Foxy or Humble) or ROS1 (Melodic/Noetic)
- C++14 or later
- [nlohmann/json](https://github.com/nlohmann/json) (auto-fetched in CMake)
- Placo, Eigen, TF2, and other dependencies (see CMakeLists.txt and package.xml)
- PicoXR Robot SDK (expected at `/opt/apps/roboticsservice/SDK`)

### Example install for ROS2 Foxy

```sh
sudo apt install ros-foxy-rclcpp ros-foxy-tf2
# For other dependencies, see CMakeLists.txt and install as needed
```

### Build (ROS2)

```sh
cd ros_ws
colcon build
source install/setup.bash
```

### Build (ROS1)

```sh
cd ros_ws
catkin_make
source devel/setup.bash
```

---

## Usage

### Run XR Publisher Node

**ROS2:**
```sh
ros2 run picoxr talker
```

**ROS1:**
```sh
rosrun picoxr publisher
```

### Run Example Application

**ROS2:**
```sh
ros2 run arx_ros2 main_v1   # publish end pose
ros2 run arx_ros2 main_v2   # publish joints pose
ros2 run arx_ros2 urdf_viz  # send robot pose data to remove `urdf-viz` server.
```

**ROS1:**
```sh
rosrun arx_ros1 main_v1
rosrun arx_ros1 main_v2
```

---

## Message Definitions

**xr_msgs/Custom.msg**
```text
int64 timestamp_ns
int32 input
Head head
Controller left_controller
Controller right_controller
```

**xr_msgs/Head.msg**
```text
float32[7] pose
int32 status
```

**xr_msgs/Controller.msg**
```text
float32 axis_x
float32 axis_y
bool axis_click
float32 gripper
float32 trigger
bool primary_button
bool secondary_button
bool menu_button
float32[7] pose
int32 status
```

---

## Assets

- **URDF/MJCF models** for ARX and UR5e robots.
- **Mesh files** for simulation and visualization.
- **Conversion scripts**: `urdf_to_mjcf.py`, `mjcf_to_urdf.py`.
- **Example scenes** for Mujoco and other simulators.

---

## License

- Main code: Apache License 2.0 (see LICENSE)
- Some assets: See individual asset directories for their licenses.