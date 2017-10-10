#!/bin/bash
rosnode kill /move_base
count=10
while [ $count -gt 0 ]
do
echo "restart robot after" $count "seconds..."
count=$((count-1))
sleep 1s
done
roslaunch gobot_navigation navigationAll.launch