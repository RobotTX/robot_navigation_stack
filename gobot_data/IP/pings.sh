#!/bin/bash
isAlive="$1"
#var=$(ifconfig | grep -A 1 wlxe | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
#if [ -z "$var" ]
#then
#    var=$(ifconfig | grep -A 1 wlan | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
#fi
#if [ -z "$var" ]
#then
#    var=$(ifconfig | grep -A 1 wlp4 | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
#fi
#var="$var.0/24"
#echo $var
#fping -r 0 -g $var 2>/dev/null | grep alive > tmp.txt
#cut -f 1 tmp.txt --delimiter=' ' > isAlive.txt
#rm tmp.txt
echo 192.168.100.29 > $isAlive
