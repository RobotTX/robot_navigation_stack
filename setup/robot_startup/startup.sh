#!/bin/bash
var=$(rosclean check | cut -d 'M' -f1 | cut -d 'K' -f1)
if [ "$var" -gt 1025 ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"M"
fi
if [ -z "$(rosnode list | grep rosout)" ] 
then
echo "start roscore"
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
sleep 3s
fi
if [ -z "$(rosnode list | grep move_base)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch;exec bash;"
fi
if [ -z "$(rosnode list | grep ping_server_new)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_software gobot_software.launch;exec bash;"
fi