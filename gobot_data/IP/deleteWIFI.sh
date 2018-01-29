#!/bin/bash
#IFS=$'\n'
wififile="$1"
wifi=$(nmcli device status | grep wifi |cut -d ' ' -f1)
n=1
while read line
do
    if [ $n -eq 1 ]
    then
        wifiname="$line"
    else
        wifipassword="$line"
    fi
    n=2 
done  < $wififile
if [ -z "$wifiname" ]
then 
    nmcli device disconnect $wifi
else
    if [ "$(nmcli connection show | grep "$wifiname")" ]
    then
        nmcli connection delete "$wifiname"
        echo Deleted WIFIs name:$wifiname
    fi
fi
