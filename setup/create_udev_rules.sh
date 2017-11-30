#!/bin/bash
#udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep KERNELS | grep "-" | grep ":" | cut -d '"' -f2
path=$(cd `dirname $0`; pwd)
echo "start copy sensors.rules to  /etc/udev/rules.d/"
echo "$path/sensors.rules"
sudo cp $path/sensors.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
echo "#################################"
echo "finish "
