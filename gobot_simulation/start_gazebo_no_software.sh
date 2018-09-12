#!/bin/bash
slam="$1"
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
    if [ "$slam" ]&&[ $slam = "slam" ]
    then
        echo "start gazebo_scan"
        gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_scan2.launch;exec bash;"
    else
        echo "start gazebo_nav"
        gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_nav.launch;exec bash;"
    fi
fi