#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch robot_hardware_test hardware_check.launch;exec bash;"