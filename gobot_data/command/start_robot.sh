#!/bin/bash
sleep 10s
source /opt/ros/kinetic/setup.bash
#clear ROS log when it is over 1GB
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
fi
#set speaker volume to be 80%
amixer -M set Master 80%
#copy the lastest log files, and clear them to store new logs
sudo sh ~/catkin_ws/src/gobot_navigation_stack/gobot_data/command/robot_log.sh
sleep 15s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_system.launch >> ~/catkin_ws/src/gobot_navigation_stack/robot_log/system_log.txt"
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch >> ~/catkin_ws/src/gobot_navigation_stack/robot_log/navigation_log.txt"
wmctrl -k on