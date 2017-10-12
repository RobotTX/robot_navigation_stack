#!/bin/bash
rosnode kill /move_base
rosnode kill //software_ping_server_new
count=10
while [ $count -gt 0 ]
do
echo "restart robot after" $count "seconds..."
count=$((count-1))
sleep 1s
done
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_software software.launch"
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation navigation.launch;exec bash;"