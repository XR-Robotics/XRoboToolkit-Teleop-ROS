# PICOXR Teleoperate ARX in ROS2

This tutorial explains how to teleoperate ARX X7S robot using PICO XR device.

## Overview

This package provides ROS2 nodes for teleoperating the ARX X7S dual-arm robot using PicoXR devices. It includes:

- **main_v1**: Direct end-effector pose control (publishes to `/ARX_VR_L` and `/ARX_VR_R`)
- **main_v2**: Joint-level control with inverse kinematics using Placo solver (publishes to `/joint_control` and `/joint_control2`)
- **urdf_viz**: Visualization node for controlling robot in [urdf-viz](https://github.com/openrr/urdf-viz).

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
- **nlohmann/json**: JSON parsing library, downloaded by `FetchContent`
- **httplib**: HTTP client library, optional for `urdf-viz`, downloaded by `FetchContent`

**Install Eigen3 using apt:**
```bash
# Install Eigen3 development package
sudo apt update
sudo apt install libeigen3-dev

# Verify installation
pkg-config --modversion eigen3
```

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

#### Prerequisites for Placo

```bash
# Install system dependencies
sudo apt update
sudo apt install -y build-essential cmake git python3-dev python3-pip

# Install Python dependencies
pip3 install numpy scipy
```

#### Compile and Install Placo

```bash
# Clone Placo repository
cd $HOME/code/github
git clone https://github.com/Rhoban/placo/
cd placo

# Install Placo requirements
bash scripts/requirements.sh

# Build Placo
mkdir -p build/host && cd build/host
cmake ../.. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Source environment (add to ~/.bashrc for permanent setup)
source ~/.bashrc
```

#### Verify Placo Installation

```bash
# Check if Placo library exists
ls -la $HOME/code/github/placo/build/host/

# Should see files like:
# liblibplaco.so (Linux) or liblibplaco.dylib (macOS)
```

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

### Remote Control Scripts

The project provides two automated scripts for remote control of the ARX X7S robot:

#### 1. End-Effector Control Script (`remote_X7s_ee.sh`)

This script launches the complete system for end-effector pose control.

**Prerequisites:**
```bash
# Set X7s_HOME environment variable to your X7s installation path
export X7s_HOME=/path/to/your/X7s

# Make script executable
chmod +x remote_X7s_ee.sh
```

**Usage:**
```bash
# Run the end-effector control script
./remote_X7s_ee.sh
```

**What it launches:**
- **CAN buses**: arx_can0.sh, arx_can1.sh, arx_can5.sh
- **Body controller**: arx_lift_controller x7s.launch.py
- **Left arm**: arx_x7_controller left_arm.launch.py
- **Right arm**: arx_x7_controller right_arm.launch.py
- **PicoXR service**: PicoXR Robot SDK service
- **XR data publisher**: picoxr talker
- **ARX control**: arx_ros2 main_v1 (end-effector control)
- **Topic monitors**: /joint_control, /ARX_VR_L
- **Network info**: IP address display

#### 2. Joint-Level Control Script (`remote_X7s_joints.sh`)

This script launches the complete system for joint-level control with inverse kinematics.

**Prerequisites:**
```bash
# Set X7s_HOME environment variable to your X7s installation path
export X7s_HOME=/path/to/your/X7s

# Make script executable
chmod +x remote_X7s_joints.sh
```

**Usage:**
```bash
# Run the joint-level control script
./remote_X7s_joints.sh
```

**What it launches:**
- **CAN buses**: arx_can0.sh, arx_can1.sh, arx_can5.sh
- **Body controller**: arx_lift_controller x7s.launch.py
- **Left arm**: arx_x7_controller left_arm_inference.launch.py
- **Right arm**: arx_x7_controller right_arm_inference.launch.py
- **PicoXR service**: PicoXR Robot SDK service
- **XR data publisher**: picoxr talker
- **ARX control**: arx_ros2 main_v2 (joint-level control with IK)
- **Topic monitors**: /joint_control, /ARX_VR_L
- **Network info**: IP address display

#### Key Differences Between Scripts

| Feature | remote_X7s_ee.sh | remote_X7s_joints.sh |
|---------|------------------|----------------------|
| **Control Mode** | End-effector pose | Joint positions with IK |
| **ARX Node** | main_v1 | main_v2 |
| **Arm Controllers** | left_arm.launch.py | left_arm_inference.launch.py |
| | right_arm.launch.py | right_arm_inference.launch.py |
| **IK Solver** | No | Placo IK solver |
| **Topics** | /ARX_VR_L, /ARX_VR_R | /joint_control, /joint_control2 |

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
