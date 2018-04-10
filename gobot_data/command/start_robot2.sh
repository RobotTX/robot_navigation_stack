#!/bin/bash
username="$1"
sleep 10s
source /opt/ros/kinetic/setup.bash
source /home/$username/catkin_ws/devel/setup.bash
#clear ROS log when it is over 1GB
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
fi
#set speaker volume to be 80%
amixer -M set Master 80%
path="/home/$username/catkin_ws/src/robot_navigation_stack"
#copy the lastest log files, and clear them to store new logs
sudo sh $path/gobot_data/command/robot_log.sh $username
sleep 15s
roslaunch gobot_navigation gobot_system.launch >> $path/robot_log/system_log.txt &
sleep 5s
roslaunch gobot_navigation gobot_navigation.launch >> $path/robot_log/navigation_log.txt &