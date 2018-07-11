#!/bin/bash
username="$1"
sleep 20s
source /opt/ros/kinetic/setup.bash
source /home/$username/catkin_ws/devel/setup.bash
#clear ROS log when it is over 1GB
if [ $(rosclean check | grep 'G') ] 
then
    echo "y" | rosclean purge
fi
#set audio output device
audio_device="alsa_output.usb-Generic_USB2.0_Device_20130100ph0-00.analog-stereo"
if [ "$(pacmd list | grep $audio_device)" ]
then
    pacmd set-default-sink $audio_device
fi
path="/home/$username/catkin_ws/src/robot_navigation_stack"
#copy the lastest log files, and clear them to store new logs
sudo sh $path/gobot_data/command/robot_log.sh $username
sleep 5s
roslaunch gobot_navigation gobot_system.launch >> $path/robot_log/system_log.txt &
sleep 5s
roslaunch gobot_navigation gobot_navigation.launch >> $path/robot_log/navigation_log.txt &