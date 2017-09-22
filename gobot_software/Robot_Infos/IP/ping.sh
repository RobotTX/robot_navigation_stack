#!/bin/bash
var=$(/sbin/ifconfig $(ip link show | grep wlan | cut -d: -f2 | awk '{print $1}') | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}' |  cut -f -3 --delimiter='.')
var="$var.0/24"
fping -r 0 -g $var 2>/dev/null | grep alive > alive.txt
cut -f 1 alive.txt --delimiter=' ' >isAlive.txt
rm alive.txt
