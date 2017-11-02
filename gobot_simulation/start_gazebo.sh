#!/bin/bash
var0=$(rosnode list | grep rosout)
if [ $? -ne 0 ] 
then
echo "start roscore"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roscore;exec bash;"
sleep 5s
fi

path=$(pwd | cut -d '/' -f1-3)

var0=$(rosnode list | grep battery_controller)
if [ $? -ne 0 ]
then
echo "start gobot_world"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source $path/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gobot_world.launch;exec bash;"
sleep 10s
fi
var0=$(rosnode list | grep move_base)
if [ $? -ne 0 ]
then
echo "start gazebo_slam"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source $path/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gazebo_slam.launch;exec bash;"
fi
var0=$(rosnode list | grep ping_server_new)
if [ $? -ne 0 ]
then
echo "start gazebo_software"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source $path/catkin_ws/devel/setup.bash;roslaunch gobot_software gazebo_software.launch;exec bash;"
fi