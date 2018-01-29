#!/bin/bash
#IFS=$'\n'
wififile="$1"
defalutwifi="Robot_Hotspot_000"
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
echo $wifiname $wifipassword
if [ -z "$wifiname" ]  #if no assigned wifi, build hotspot
then
    if [ -z "$(nmcli device status | grep Hotspot)" ]  #if no created hotspot, build one
    then
        if [ -z "$(nmcli connection show | grep Hotspot)" ]
        then
            nmcli d wifi hotspot ssid "$defalutwifi" password "robotics"
            echo "ResetWifi: Robot build its own wifi named '$defalutwifi'"
        else
            nmcli connection up "Hotspot"
            echo "ResetWifi: Robot connecting to its own wifi named '$defalutwifi'"
        fi
    #else
        #echo "Robot connected to its own wifi named '$defalutwifi' as there is no assgined wifi"
    fi
else      
    if [ -z "$(nmcli device status | grep "$wifiname")" ] #if has assigned wifi, but not connected
    then
        if [ -z "$(nmcli connection show | grep "$wifiname")" ]  #if not in the connection list, build new connection
        then
            sleep 3s
            nmcli d wifi connect "$wifiname" password "$wifipassword"
            echo "ResetWifi: Robot build connection to assigned wifi named '$wifiname'"
        else
            nmcli connection up "$wifiname"
            echo "ResetWifi: Robot connecting to assigned wifi named '$wifiname'"
        fi
    fi
fi