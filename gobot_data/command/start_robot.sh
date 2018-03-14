#!/bin/bash
sleep 10s
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
fi
sleep 20s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_base gobot_base.launch;exec bash;"