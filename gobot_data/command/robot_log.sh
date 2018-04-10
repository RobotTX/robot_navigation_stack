#!/bin/bash
path="catkin_ws/src/robot_navigation_stack/robot_log"
cp ~/$path/system_log.txt ~/$path/old_log/system_old_log.txt
cp ~/$path/navigation_log.txt ~/$path/old_log/navigation_old_log.txt
DATE=`date '+%Y-%m-%d %H:%M:%S'`
echo "Log start at: $DATE" > ~/$path/system_log.txt
echo "Log start at: $DATE" > ~/$path/navigation_log.txt