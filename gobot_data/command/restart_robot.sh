#!/bin/bash
sleep 10s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rosnode kill -a"
sleep 3s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_system.launch;exec bash;"
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch;exec bash;"