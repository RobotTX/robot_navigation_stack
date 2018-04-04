#!/bin/bash
var0=$(rosnode list | grep rosout)
if [ $? -ne 0 ] 
then
echo "start roscore"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
sleep 3s
fi
var0=$(rosnode list | grep battery_controller)
if [ $? -ne 0 ]
then
echo "start gobot_world"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gobot_world.launch;exec bash;"
sleep 15s
fi
var0=$(rosnode list | grep move_base)
if [ $? -ne 0 ]
then
echo "start gazebo_slam"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_slam.launch;exec bash;"
fi
var0=$(rosnode list | grep software_ping_servers)
if [ $? -ne 0 ]
then
echo "start gazebo_software"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_software.launch;exec bash;"
fi