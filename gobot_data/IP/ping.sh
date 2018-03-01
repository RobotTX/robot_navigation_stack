#!/bin/bash
isAlive="$1"
wififile="$2"
defalutwifi="Robot_Hotspot_00X"
wifi=$(nmcli device status | grep wifi |cut -d ' ' -f1)
n=1
#read wifi infomation from local file
#nmcli device wifi list | grep -w "$wifiname"
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
echo -n "" > $isAlive #clear recorded servers' IP
#echo "(Ping_Servers)" name:$wifiname password:$wifipassword
if [ -z "$wifiname" ]  #if no assigned wifi, build hotspot
then
    if [ -z "$(nmcli device status | grep -w Hotspot)" ]  #if no created hotspot, build one
    then
        if [ -z "$(nmcli connection show | grep -w Hotspot)" ]
        then
            nmcli d wifi hotspot ssid "$defalutwifi" password "robotics"
            echo "(Ping_Servers) Robot build its own wifi named '$defalutwifi'"
        else
            nmcli connection up "Hotspot"
            echo "(Ping_Servers) Robot connecting to its own wifi named '$defalutwifi'"
        fi
    #else
        #echo "(Ping_Servers) Robot connected to its own wifi named '$defalutwifi' as there is no assgined wifi"
    fi
else 
    if [ "$(nmcli device wifi list | grep -w "$wifiname")" ] #if able to find wifi in the list
    then     
        if [ -z "$(nmcli device status | grep -w "$wifiname")" ] #if has assigned wifi, but not connected
        then
            if [ -z "$(nmcli connection show | grep -w "$wifiname")" ]  #if not in the connection list, build new connection
            then
                if [ -z "$(nmcli device status | grep wifi | grep disconnected)" ]
                then
                    nmcli device disconnect $wifi
                    echo "(Ping_Servers) Disconnect current wifi"
                fi
                    nmcli d wifi connect "$wifiname" password "$wifipassword"
                    echo "(Ping_Servers) Robot build connection to assigned wifi named '$wifiname'"
            else
                nmcli connection up "$wifiname"
                echo "(Ping_Servers) Robot connecting to assigned wifi named '$wifiname'"
            fi
        #else
            #echo "(Ping_Servers) Robot connected to assigned wifi named '$wifiname'"
        fi
    else
        echo "(Ping_Servers) Unable to find assigned wifi:'$wifiname' in the scan list"
    fi
fi
#record all available hosts in the server
var=$(ifconfig | grep -A 1 $wifi | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
fping -r 0 -g "$var.0/24" 2>/dev/null | grep alive | cut -d ' ' -f1 > $isAlive
#delete servers that we don't want to connect
#sed -i "/$var.ip/d" $isAlive
#check USB tethering connection
usb_tether=$(route -n | grep enp | grep UG | cut -d ' ' -f10)
if [ "$usb_tether" ]
then
    #echo "(Ping_Servers) Found USB connection to tablet..."
    echo $usb_tether >> $isAlive
fi
