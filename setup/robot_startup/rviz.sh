#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rviz -d ~/catkin_ws/src/gobot_navigation_stack/setup/robot_startup/slam.rviz"
