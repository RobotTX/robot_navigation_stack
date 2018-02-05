#!/bin/bash
sleep 10s
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
fi
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
sleep 15s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_software gobot_software.launch;exec bash;"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch;exec bash;"