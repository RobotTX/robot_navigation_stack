#!/bin/bash
#IFS=$'\n'
wififile=$1
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
echo name:$wifiname password:$wifipassword
if [ "$wifiname" ]
then
    if [ "$(nmcli connection show | grep "$wifiname")" ]
    then
        nmcli connection delete "$wifiname"
        echo "delete previous assigned wifi $wifiname"
    fi
fi