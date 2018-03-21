#!/bin/bash
path="catkin_ws/src/gobot_navigation_stack/robot_log"
cp ~/$path/base_log.txt ~/$path/old_log/base_old_log.txt
cp ~/$path/navigation_log.txt ~/$path/old_log/navigation_old_log.txt
cp ~/$path/software_log.txt ~/$path/old_log/software_old_log.txt
DATE=`date '+%Y-%m-%d %H:%M:%S'`
echo "Log start at: $DATE" > ~/$path/base_log.txt
echo "Log start at: $DATE" > ~/$path/navigation_log.txt
echo "Log start at: $DATE" > ~/$path/software_log.txt