#!/bin/bash
#udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep KERNELS | grep "-" | grep ":" | cut -d '"' -f2
path=$(cd `dirname $0`; pwd)
STM="x-x.x:x.0"
MD="x-x.x:x.1"
if [ "$(ls -l /dev/ttyUSB1 | grep root)" ] 
then
    STMnew=$(udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB1) | grep KERNELS | grep "-" | grep ":" | cut -d '"' -f2)
    sed -i "s/$STM/$STMnew/g" `grep $STM -rl $path/sensors.rules`
    echo  "Replaced udev_rules USB1 port：$STM!"
else
    echo  "Can not find USB1 port for STM!"
fi
if [ "$(ls -l /dev/ttyUSB2 | grep root)" ] 
then
    MDnew=$(udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB2) | grep KERNELS | grep "-" | grep ":" | cut -d '"' -f2)
    sed -i "s/$MD/$MDnew/g" `grep $MD -rl $path/sensors.rules`
    echo  "Replaced udev_rules USB2 port:$MD!"
else
    echo "Can not find USB2 port for MD!"
fi
echo "start copy sensors.rules to  /etc/udev/rules.d/"
echo "$path/sensors.rules"
sudo cp $path/sensors.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
echo "#################################"
echo "finish "
