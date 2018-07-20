#!/bin/bash
sleep 20s
source /opt/ros/kinetic/setup.bash
#clear ROS log when it is over 1GB
log_data=$(rosclean check | grep 'G')
if [ "$log_data" ]
then
    echo "y" | rosclean purge
    echo "Cleaned ros log data:"$log_data "G"
fi
#set audio output device
audio_device="alsa_output.usb-Generic_USB2.0_Device_20130100ph0-00.analog-stereo"
if [ "$(pacmd list | grep $audio_device)" ]
then
    pacmd set-default-sink $audio_device
fi
#copy the lastest log files, and clear them to store new logs
sudo sh ~/catkin_ws/src/robot_navigation_stack/gobot_data/command/robot_log.sh
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_system.launch >> ~/catkin_ws/src/robot_navigation_stack/robot_log/system_log.txt"
sleep 5s
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch >> ~/catkin_ws/src/robot_navigation_stack/robot_log/navigation_log.txt"
wmctrl -k on