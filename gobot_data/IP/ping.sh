#!/bin/bash
isAlive="$1"
wififile="$2"
defalutwifi="Robot_Hotspot_"
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
#echo name:$wifiname password:$wifipassword
if [ -z "$wifiname" ]  #if no assigned wifi, build hotspot
then
    if [ -z "$(nmcli device status | grep Hotspot)" ]  #if no created hotspot, build one
    then
        if [ -z "$(nmcli connection show | grep Hotspot)" ]
        then
            nmcli d wifi hotspot ssid "$defalutwifi" password "robotics"
            echo "Robot build its own wifi named '$defalutwifi'"
        else
            nmcli connection up "Hotspot"
            echo "Robot connecting to its own wifi named '$defalutwifi'"
        fi
    #else
        #echo "Robot connected to its own wifi named '$defalutwifi' as there is no assgined wifi"
    fi

else      
    if [ -z "$(nmcli device status | grep "$wifiname")" ] #if has assigned wifi, but not connected
    then
        if [ -z "$(nmcli connection show | grep "$wifiname")" ]  #if not in the connection list, build new connection
        then
            if [ -z "$(nmcli device status | grep wifi | grep disconnected)" ]
            then
                nmcli device disconnect $wifi
                echo "Disconnect current wifi"
                sleep 5s
            fi
                nmcli d wifi connect "$wifiname" password "$wifipassword"
                echo "Robot build connection to assigned wifi named '$wifiname'"
        else
            nmcli connection up "$wifiname"
            echo "Robot connecting to assigned wifi named '$wifiname'"
        fi
    #else
        #echo "Robot connected to assigned wifi named '$wifiname'"
    fi
fi
var=$(ifconfig | grep -A 1 $wifi | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
fping -r 0 -g "$var.0/24" 2>/dev/null | grep alive | cut -d ' ' -f1 > $isAlive
sed -i "/$var.29/d" $isAlive
#sed -i "/$var.165/d" $isAlive