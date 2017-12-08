#!/bin/bash
rosnode kill /move_base
rosnode kill //software_ping_server_new
sleep 3s
sudo shutdown -r now
#sudo reboot