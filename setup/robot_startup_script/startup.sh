#!/bin/bash
source /opt/ros/kinetic/setup.bash
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"G"
fi
if [ -z "$(rosnode list | grep ping_server_new)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_software gobot_software.launch"
    sleep 3s
fi
if [ -z "$(rosnode list | grep base_sensors)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_base gobot_base.launch"
    sleep 5s
fi
if [ -z "$(rosnode list | grep move_base)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch"
fi