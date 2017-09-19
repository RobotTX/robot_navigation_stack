#!/bin/bash
rm tmp.txt

    while read line  
    do
	if [[ $line == *"wpa-ssid"* ]] && [[ $line != *"#"* ]] ; then
		echo 'wpa-ssid "'$1'"' >> tmp.txt
	elif [[ $line == *"wpa-psk"* ]] && [[ $line != *"#"* ]] ; then
		echo 'wpa-psk "'$2'"' >> tmp.txt
	else
		echo $line >> tmp.txt
	fi
    done </etc/network/interfaces

mv tmp.txt /etc/network/interfaces

#use : ./change_wifi.sh "wifi_name" pwd

wlan=$(ip link show | grep wlan | cut -d: -f2 | awk '{print $1}')
echo $wlan

ifdown $wlan
ifup $wlan &

start=$(date +%s);

diff=0
while  ! (echo `fping www.google.com` | grep -q "alive" ) && [[ $diff -lt 5 ]] ;
do 
	end=$(date +%s);
	diff=`echo $((end-start)) | awk '{print($1%60)}'`
	echo $diff
	#echo 'not connected'
done 

if !(echo `fping www.google.com` | grep -q "alive") ; then

val=`ps -ef | grep "ifup $wlan" | head -n 1| cut -d " " -f 6`
echo `ps -ef | grep "ifup $wlan" | head -n 1`
echo $val
kill -9 $val

fi


 
	
