#!/bin/bash
isAlive="$1"
var=$(/sbin/ifconfig $(ip link show | grep wlan | cut -d: -f2 | awk '{print $1}') | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}' |  cut -f -3 --delimiter='.')
if [[ -z "$var" ]]
    var=$(/sbin/ifconfig $(ip link show | grep wlxe | cut -d: -f2 | awk '{print $1}') | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}' |  cut -f -3 --delimiter='.')
fi
if [[ -z "$var" ]]
    var=$(/sbin/ifconfig $(ip link show | grep wlp4 | cut -d: -f2 | awk '{print $1}') | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}' |  cut -f -3 --delimiter='.')
fi
var="$var.0/24"
fping -r 0 -g $var 2>/dev/null | grep alive > tmp.txt
cut -f 1 tmp.txt --delimiter=' ' >$isAlive
#echo 127.0.0.1 >> $isAlive
rm tmp.txt
