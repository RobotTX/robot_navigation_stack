#!/bin/bash
var0=$(ps -a | grep roscore)
if [ $? -ne 0 ] 
then
    var=$(rosclean check | cut -d 'M' -f1)
    if [ "$var" -gt 1000 ] 
    then
        echo "y" | rosclean purge
        echo "Cleaned ros log data:"$var"M"
    fi
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
    sleep 3s
fi
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_base mapbase.launch;exec bash;"