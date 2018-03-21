#!/bin/bash
#udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep KERNELS | grep "-" | grep ":" | cut -d '"' -f2
echo "USB0: "$(udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep -E 'idProduct|bInterfaceNumber')
echo "USB1: "$(udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB1) | grep -E 'idProduct|bInterfaceNumber')  
echo "USB2: "$(udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB2) | grep -E 'idProduct|bInterfaceNumber') 
path=$(cd `dirname $0`; pwd)
echo "start copy $path/sensors.rules to  /etc/udev/rules.d/"
sudo cp $path/sensors.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
echo "#################################"
echo "finish "
