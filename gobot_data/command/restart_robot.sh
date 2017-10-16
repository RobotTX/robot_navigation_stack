#!/bin/bash
rosnode kill /move_base
rosnode kill //software_ping_server_new
sleep 10s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_software gobot_software.launch"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch"