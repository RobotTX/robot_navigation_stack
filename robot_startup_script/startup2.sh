#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"G"
fi
sudo sh ~/catkin_ws/src/robot_navigation_stack/gobot_data/command/robot_log.sh
sleep 2s
roslaunch gobot_navigation gobot_system.launch >> $path/robot_log/system_log.txt &
sleep 5s
roslaunch gobot_navigation gobot_navigation.launch >> ~/catkin_ws/src/robot_navigation_stack/robot_log/navigation_log.txt &