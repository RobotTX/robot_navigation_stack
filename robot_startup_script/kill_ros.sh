#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rosnode kill -a;sleep 5s"