#!/bin/bash
sleep 10s
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
fi
#stop network-manager to avoid laser shutdown problems
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
sleep 20s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_base gobot_base.launch;exec bash;"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_software gobot_software.launch;exec bash;"
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch;exec bash;"