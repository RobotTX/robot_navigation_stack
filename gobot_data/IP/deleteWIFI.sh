#!/bin/bash
#IFS=$'\n'
echo "######################Delete Wifi Start######################"
oldWifiName="$1"
newWifiName="$2"
newWifiPasswd="$3"
wifi=$(nmcli device status | grep wifi |cut -d ' ' -f1)
#delete old wifi information
nmcli device disconnect $wifi    #disconnect current wifi
if [ "$oldWifiName" = "null_wifi" ]   #if old wifi is Hotspot
then
    echo "(DELETE WIFI) Disconnect robot wifi: #Hotspot#"
else        #if old wifi is not Hotspot, then delete it from wifi memory list
    echo "(DELETE WIFI) Disconnect robot wifi: #$oldWifiName#"
    if [ "$(nmcli connection show | grep -w "$oldWifiName")" ]
    then
        nmcli connection delete $oldWifiName
        echo "(DELETE WIFI) Deleted WIFIs name: #$oldWifiName#"
    fi
fi
sleep 1s
if [ "$newWifiName" = "null_wifi" ]   #if new wifi is Hotspot
then
    nmcli connection up "Hotspot"
    echo "(DELETE WIFI) Robot connected to its own wifi"
else                            #if new wifi is not Hotspot, make a new connetion to it
    if [ -z "$(nmcli device wifi list | grep -w "$newWifiName")" ]      #if can not find new wifi in scan list, then restart network-manager
    then
        sudo service network-manager restart 
        echo "(DELETE WIFI) Restart network-manager as assigned "
        sleep 1s
    fi
    nmcli d wifi connect "$newWifiName" password "$newWifiPasswd"
    echo "(DELETE WIFI) Robot build connection to assigned wifi named #$newWifiName#"
fi
echo "######################Delete Wifi End######################"
