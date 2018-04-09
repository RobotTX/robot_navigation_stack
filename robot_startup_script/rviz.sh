#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rviz -d ~/catkin_ws/src/gobot_navigation_stack/robot_startup_script/slam.rviz"
