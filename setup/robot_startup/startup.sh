#!/bin/bash
source /opt/ros/kinetic/setup.bash
var=$(rosclean check | grep 'G')
if [ ! -z "$var" ] 
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$var"G"
fi
sudo sh ~/catkin_ws/src/gobot_navigation_stack/gobot_data/command/robot_log.sh
if [ -z "$(rosnode list | grep ping_server_new)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_software gobot_software.launch >> ~/catkin_ws/src/gobot_navigation_stack/robot_log/software_log.txt"
fi
sleep 3s
if [ -z "$(rosnode list | grep base_sensors)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_base gobot_base.launch >> ~/catkin_ws/src/gobot_navigation_stack/robot_log/base_log.txt"
fi
sleep 5s
if [ -z "$(rosnode list | grep move_base)" ]
then
    gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch >> ~/catkin_ws/src/gobot_navigation_stack/robot_log/navigation_log.txt"
fi