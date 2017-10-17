#!/bin/bash
isAlive="$1"
var=$(ifconfig | grep -A 1 wlxe | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
if [ -z "$var" ]
then
    var=$(ifconfig | grep -A 1 wlan | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
fi
if [ -z "$var" ]
then
    var=$(ifconfig | grep -A 1 wlp4 | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
fi
echo $var
fping -r 0 -g "$var.0/24" 2>/dev/null | grep alive | cut -d ' ' -f1 > $isAlive
sed -i "/$var.33/d" $isAlive