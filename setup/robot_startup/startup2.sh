#!/bin/bash
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"G"
fi
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch gobot_software gobot_software.launch &
sleep 3s
roslaunch gobot_base gobot_base.launch &
sleep 5s
roslaunch gobot_navigation gobot_navigation.launch &