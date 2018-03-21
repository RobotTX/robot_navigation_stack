#!/bin/bash
#udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep KERNELS | grep "-" | grep ":" | cut -d '"' -f2
#path=$(cd `dirname $0`; pwd)
#echo "start copy $path/rc.local to /etc/"
#sudo cp $path/rc.local  /etc
#echo "#################################"
#echo "finish "
