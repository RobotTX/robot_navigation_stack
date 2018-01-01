#!/bin/bash
#linux sed 批量替换多个文件中的字符串
#sed -i "s/oldstring/newstring/g" `grep oldstring -rl yourdir`
defaultname="gtdollar"
username=$(whoami)
path=$(pwd | cut -d '/' -f1-3)
echo "Current Username: $username; Root Path: $path"
sed -i "s/$defaultname/$username/g" `grep $defaultname -rl $path/catkin_ws/src/gobot_navigation_stack`
echo "Username has been changed!"
echo "#################################"