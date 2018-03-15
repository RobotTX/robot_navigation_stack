#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rosnode kill /move_base /base_sensors /motor_wheels"
sleep 10s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_base gobot_base.launch;exec bash;"
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch;exec bash;"