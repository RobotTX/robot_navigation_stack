#!/bin/bash
sleep 10s
var=$(rosclean check | cut -d 'M' -f1)
if [ "$var" -gt 1000 ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"M"
fi
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation navigation.launch;exec bash;"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_software software.launch;exec bash;"