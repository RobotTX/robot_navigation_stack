#!/bin/bash
sleep 10s
path=$(pwd | cut -d '/' -f1-3)
var=$(rosclean check | cut -d 'M' -f1 | cut -d 'K' -f1)
if [ "$var" -gt 1000 ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"M"
fi
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source $path/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch;exec bash;"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source $path/catkin_ws/devel/setup.bash;roslaunch gobot_software gobot_software.launch;exec bash;"