#!/bin/bash
var=$(rosclean check | cut -d 'M' -f1 | cut -d 'K' -f1)
if [ "$var" -gt 1025 ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"M"
fi
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore &
sleep 3s
roslaunch gobot_navigation gobot_navigation.launch &
roslaunch gobot_software gobot_software.launch &