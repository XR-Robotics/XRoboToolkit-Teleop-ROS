# PicoXR Teleoperate ARX in ROS2

This tutorial explains how to teleoperate ARX X7S robot using PICO XR device.

## Overview

This package provides ROS2 nodes for teleoperating the ARX X7S dual-arm robot using PicoXR devices. It includes:

- **main_v1**: Direct end-effector pose control (publishes to `/ARX_VR_L` and `/ARX_VR_R`)
- **main_v2**: Joint-level control with inverse kinematics using Placo solver (publishes to `/joint_control` and `/joint_control2`)
- **urdf_viz**: Visualization node for controlling robot in URDF visualization server

## Prerequisites

### System Requirements
- Ubuntu 20.04 (for ROS2 Foxy) or Ubuntu 22.04 (for ROS2 Humble)
- ROS2 Foxy or Humble

### Dependencies

#### Core ROS2 Dependencies
```bash
# For ROS2 Foxy
sudo apt install ros-foxy-rclcpp ros-foxy-tf2 ros-foxy-std-msgs

# For ROS2 Humble
sudo apt install ros-humble-rclcpp ros-humble-tf2 ros-humble-std-msgs
```

#### Additional Dependencies
The project uses several external libraries that are automatically fetched during build:
- **Eigen3**: Linear algebra library
- **Placo**: Kinematics solver library
- **nlohmann/json**: JSON parsing library
- **httplib**: HTTP client library, optional for `urdf-viz`.

#### Custom Message Dependencies
- `arm_control`: Robot control messages
- `xr_msgs`: XR device messages

### PicoXR Setup
- Install PicoXR Robot SDK (expected at `/opt/apps/roboticsservice/SDK`)
- Ensure PicoXR device is connected and configured

## Environment Setup

### 1. Clone and Build the Project

```bash
# Clone the repository
git clone https://github.com/XR-Robotics/XRoboToolkit-Teleop-ROS
git clone https://github.com/ARXroboticsX/X7s
cd XRoboToolkit-Teleop-ROS

# create workspace
mkdir -p ws_ros2/src
cd ws_ros2/src
ln -s ../../ros2/picoxr
ln -s ../../ros2/xr_msgs
ln -s ../../examples/arx_ros2
ln -s ../../cmake
ln -s ../../../X7s/x7s/ROS2/x7s_ws/src/arxmsgros2/arm_control
cd ..
cp ../examples/arx_ros2/scripts/env_humble env
cp ../examples/arx_ros2/scripts/remote_X7s_ee.sh .
cp ../examples/arx_ros2/scripts/remote_X7s_joints.sh .
source ./env
```

### 2. Install Dependencies

```bash
# Run the provided install script
bash examples/arx_ros2/install.sh
```

### 3. Install placo [optional]

`placo` is required for joints control with IK.

## Build Instructions

### Manual Build
```bash
# Build the project
cd path_to_ws_ros2
colcon build 
source ./env
```

### Build Options
```bash
# Build with debug information
colcon build --packages-select arx_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build with release optimization
colcon build --packages-select arx_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running the Executables

### 1. Main V1 - End-Effector Control

This version provides direct end-effector pose control without inverse kinematics.

```bash
# Run the main_v1 node
ros2 run arx_ros2 main_v1
```

**What it does:**
- Subscribes to `/xr_pose` for XR device data
- Converts XR pose to ARX end-effector pose
- Publishes position commands to `/ARX_VR_L` and `/ARX_VR_R`
- Runs at 10Hz (100ms timer)

**Topics:**
- **Subscribes:** `/xr_pose` (xr_msgs/Custom)
- **Publishes:** `/ARX_VR_L`, `/ARX_VR_R` (arm_control/PosCmd)

### 2. Main V2 - Joint-Level Control with IK

This version uses Placo inverse kinematics solver for joint-level control.

```bash
# Run the main_v2 node
ros2 run arx_ros2 main_v2
```

**What it does:**
- Subscribes to `/xr_pose` for XR device data
- Converts XR pose to ARX end-effector pose
- Uses Placo IK solver to compute joint positions
- Publishes joint commands to `/joint_control` and `/joint_control2`
- Also publishes end-effector poses for compatibility
- Runs at 10Hz (100ms timer)

**Topics:**
- **Subscribes:** `/xr_pose` (xr_msgs/Custom)
- **Publishes:** 
  - `/joint_control`, `/joint_control2` (arm_control/JointControl)
  - `/ARX_VR_L`, `/ARX_VR_R` (arm_control/PosCmd)

### 3. URDF Visualization

This node provides visualization control for URDF-based robot simulation.

```bash
# Set the URDF visualization server IP
export URDF_VIZ_IP_ADDR="localhost:8080"

# Run the urdf_viz node
ros2 run arx_ros2 urdf_viz
```

**What it does:**
- Subscribes to joint control messages
- Sends joint positions to URDF visualization server
- Controls left arm (joint5-joint11) and right arm (joint14-joint20)
- Runs at 10Hz update rate

**Topics:**
- **Subscribes:** `/joint_control`, `/joint_control2` (arm_control/JointControl)

## Usage Examples

### Basic Teleoperation Setup

1. **Start XR data publisher:**
```bash
ros2 run picoxr talker
```

2. **Start ARX control node:**
```bash
# For end-effector control
ros2 run arx_ros2 main_v1

# OR for joint-level control with IK
ros2 run arx_ros2 main_v2
```

3. **Start visualization (optional):**
```bash
export URDF_VIZ_IP_ADDR="localhost:8080"
ros2 run arx_ros2 urdf_viz
```

## Configuration

### URDF Models

The package includes several URDF models in the `urdf/` directory:
- `X7S.urdf`: Complete ARX X7S model
- `X7S_arm_only.urdf`: Arms only (used by main_v2)
- `X7S_fixed_vertical_no_gripper.urdf`: Fixed vertical configuration

### Joint Mapping

- **Left Arm:** joint5-joint11 (7 DOF)
- **Right Arm:** joint14-joint20 (7 DOF)
- **Head:** joint3, joint4

### Control Parameters

- **Update Rate:** 10Hz (100ms timer)
- **Print Interval:** Every 10 frames (main_v1), every 10 frames (main_v2), every 100 frames (urdf_viz)
- **IK Solver:** Placo with kinetic energy regularization

## Troubleshooting

### Common Issues

1. **Build Errors:**
   - Ensure all ROS2 dependencies are installed
   - Check CMake version (requires 3.8+)
   - Verify C++14 support

2. **Runtime Errors:**
   - Ensure XR device is connected and PicoXR SDK is installed
   - Check that `/xr_pose` topic is being published
   - Verify environment variables are set correctly

3. **IK Solver Issues:**
   - Check URDF file path in main_v2
   - Ensure Placo library is properly linked
   - Verify joint limits in URDF

### Debug Mode

```bash
# Enable debug logging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_LOGGING_USE_STDOUT=1

# Run with debug output
ros2 run arx_ros2 main_v2 --ros-args --log-level debug
```

## Message Types

### Input Messages
- `xr_msgs/Custom`: XR device pose and controller data
- `arm_control/JointControl`: Joint position commands

### Output Messages
- `arm_control/PosCmd`: End-effector position commands
- `arm_control/JointControl`: Joint position commands

## License

This project is licensed under the MIT License. See the main repository LICENSE file for details.
