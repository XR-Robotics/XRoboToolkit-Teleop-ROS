#!/bin/bash

X7s_HOME=
if [ -z "$X7s_HOME" ];then
    echo "X7s_HOME is not set"
    exit 1
fi

workspace=${X7s_HOME}/00-sh/ROS2

source_env () {
    source /opt/ros/humble/setup.sh
    local_shell="./install/setup.sh"
    if [ -f $local_shell ];then
        source $local_shell
    fi
    local_env=".venv/bin/activate"
    if [ -f $local_env ];then
        source $local_env
    fi
}
source_env

# CAN
gnome-terminal -t "can1" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can0.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can3" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 0.1
gnome-terminal -t "can5" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can5.sh; exec bash;"
sleep 1

#body
gnome-terminal -t "body" -x  bash -c "cd ${workspace}; cd ../..; cd body/ROS2; source install/setup.bash && ros2 launch arx_lift_controller x7s.launch.py ; exec bash;"
#x7s
sleep 3
gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd x7s/ROS2/x7s_ws; source install/setup.bash && ros2 launch arx_x7_controller left_arm_inference.launch.py; exec bash;"
sleep 0.5
gnome-terminal -t "R" -x  bash -c "cd ${workspace}; cd ../..; cd x7s/ROS2/x7s_ws; source install/setup.bash && ros2 launch arx_x7_controller right_arm_inference.launch.py; exec bash;"

# PICO VR
gnome-terminal -t "PICO" -x bash -c "bash /opt/apps/picobusinesssuite/runService.sh && cd ${HOME}/ros2_ws && source ./env && ros2 run picoxr talker; exec bash;"
sleep 1 
gnome-terminal -t "arx_ros2__main_v2" -x bash -c "cd ${HOME}/ros2_ws && source ./env && ros2 run arx_ros2 main_v2; exec bash;"
sleep 0.2
gnome-terminal -t "/joint_control" -x bash -c "cd ${HOME}/ros2_ws && source ./env && ros2 topic echo /joint_control; exec bash;"
sleep 0.2
gnome-terminal -t "/ARX_VR_L" -x bash -c "cd ${HOME}/ros2_ws && source ./env && ros2 topic echo /ARX_VR_L; exec bash;"
sleep 0.2
gnome-terminal -t "IP" -x bash -c "ifconfig | grep inet; exec bash;"
