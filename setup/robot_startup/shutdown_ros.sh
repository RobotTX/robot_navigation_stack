#!/bin/bash
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rosnode kill /move_base;rosnode kill /software_ping_server_new;rosnode kill /rosout"