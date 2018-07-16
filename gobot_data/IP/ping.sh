#!/bin/bash
isAlive="$1"
wififile="$2"
defalutwifi="Robot_Hotspot_GTD002"
connection="wifi"
#check WIFI connection
wifi=$(nmcli device status | grep wifi | cut -d ' ' -f1 | head -n 1)
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
    connection="hotspot"
else 
    if [ "$(nmcli device wifi list | grep -w "$wifiname")" ] #if able to find wifi in the list
    then     
        if [ -z "$(nmcli device status | grep -w "$wifiname")" ] #if assigned wifi is not connected
        then
            if [ -z "$(nmcli device status | grep wifi | grep disconnected)" ]  #if connect other wifi, disconnect it first
            then
                nmcli device disconnect $wifi
                echo "(PING WIFI) Disconnect current wifi"
            fi
            if [ -z "$(nmcli connection show | grep -w "$wifiname")" ]  #if not in the connection list, build new connection
            then
                    nmcli d wifi connect "$wifiname" password "$wifipassword" ifname $wifi
                    echo "(PING WIFI) Robot build connection to assigned wifi named #$wifiname#"
            else
                nmcli connection up "$wifiname" ifname $wifi
                echo "(PING WIFI) Robot connecting to assigned wifi named #$wifiname#"
            fi
        else
            echo "(PING WIFI) Robot connected to assigned wifi named #$wifiname#"
        fi
        #record all available hosts in the server
        var=$(ifconfig | grep -A 1 $wifi | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
        fping -r 0 -g "$var.0/24" 2>/dev/null | grep alive | cut -d ' ' -f1 > $isAlive
        servers=$(wc -l $isAlive | cut -d ' ' -f1)
        echo "(PING WIFI) Detected wifi connection. #$servers# servers available."
        if [ $servers -lt 2 ]  #if no. of servers smaller than 2, network has connection issue
        then
            echo "(PING WIFI) restart network-mangager due to connection issue"
            sudo service network-manager restart #restart network-manager if has issue
        fi
        #delete servers that we don't want to connect
        #sed -i "/$var.14/d" $isAlive
    else
        #if can not find assigned wifi in the list, build robot hotspot for user to connect
        echo "(PING WIFI) Unable to find assigned wifi:#$wifiname# in the scan list"
        sudo service network-manager restart  #restart network-manager if has issue
        #connection="hotspot"
    fi
fi
#if need to build hotspot
if [ $connection = "hotspot" ]
then
    if [ -z "$(nmcli device status | grep -w Hotspot)" ]  #if no created hotspot, build one
    then
        if [ -z "$(nmcli connection show | grep -w Hotspot)" ]
        then
            nmcli d wifi hotspot ssid "$defalutwifi" password "robotics" ifname $wifi
            echo "(PING WIFI) Robot build its own wifi named #$defalutwifi#"
        else
            nmcli connection up "Hotspot" ifname $wifi
            echo "(PING WIFI) Robot connecting to its own wifi named #$defalutwifi#"
        fi
    else
        echo "(PING WIFI) Robot connected to its own wifi named #$defalutwifi#"
    fi
    #record all available hosts in the server
    var=$(ifconfig | grep -A 1 $wifi | grep inet | cut -d ':' -f2|cut -f -3 --delimiter='.')
    fping -r 0 -g "$var.0/24" 2>/dev/null | grep alive | cut -d ' ' -f1 > $isAlive
fi
#check USB tethering connection
usb_device=$(nmcli device status | grep "ethernet" | grep -v "Wired Ethernet" | grep "connected" | cut -d ' ' -f1)
if [ "$usb_device" ]
then
    usb_ip=$(route -n | grep $usb_device | grep UG | cut -d ' ' -f10)
    echo $usb_ip >> $isAlive
    echo "(PING WIFI) Detected usb tethering connection"
    #echo "(PING WIFI) usb device:$usb_device $usb_ip"
fi
#check ethernet connection
ethernet_device=$(nmcli device status | grep "ethernet" | grep "Wired Ethernet" | grep "connected" | cut -d ' ' -f1)
ethernet_ip="1.2.5.2"
if [ "$ethernet_device" ]
then
    echo $ethernet_ip >> $isAlive
    echo "(PING WIFI) Detected ethernet connection"
    #echo "(PING WIFI) eth device:$ethernet_device $ethernet_ip"
fi
