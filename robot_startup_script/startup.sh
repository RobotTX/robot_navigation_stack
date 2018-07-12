#!/bin/bash
source /opt/ros/kinetic/setup.bash
log_data=$(rosclean check | grep 'G')
if [ "$log_data" ]
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$log_data "G"
fi
sleep 2s
if [ -z "$(rosnode list | grep base_sensors)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_system.launch;exec bash;"
fi
sleep 5s
if [ -z "$(rosnode list | grep move_base)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch;exec bash;"
fi