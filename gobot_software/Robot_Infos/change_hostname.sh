rm tmp.txt

echo "new hostname : $1"

echo $1 > /etc/hostname
hostname $1
cpt=0
    while read line  
    do
        if [ $cpt = 1 ] ; then
                echo "127.0.1.1 $1" >> tmp.txt
        else
                echo $line >> tmp.txt
        fi
        cpt=$((cpt + 1 ))
    done </etc/hosts

mv tmp.txt /etc/hosts

cpt=1
    while read -r line  
    do
        if [ ! -z "$(grep -n 'export ROS_HOSTNAME=' /home/ubuntu/.bashrc | grep -w $cpt)" ] && 
	[ -z "$(grep -n '#export ROS_HOSTNAME=' /home/ubuntu/.bashrc | grep -w $cpt)" ]; then
		echo "export ROS_HOSTNAME=$1" >> tmp.txt
	else
                echo $line >> tmp.txt
        fi
        cpt=$((cpt + 1 ))
    done </home/ubuntu/.bashrc

mv tmp.txt /home/ubuntu/.bashrc


line=`xauth list | tail -n1`
old_hostname=`echo $line | cut -d "/" -f 1`
add=`echo $line | sed -e "s/${old_hostname}/${1}/"`


getfacl -R  -p /home/ubuntu/.Xauthority > /home/ubuntu/backup/saved-permissions_hostname_ubuntu ;
getfacl -R -p /root/.Xauthority > /home/ubuntu/backup/saved-permissions_hostname_root ;

xauth add $add


setfacl --restore=/home/ubuntu/backup/saved-permissions_hostname_ubuntu
setfacl --restore=/home/ubuntu/backup/saved-permissions_hostname_root
rm saved-permissions_hostname_ubuntu
rm saved-permissions_hostname_root
rm -fr /home/ubuntu/.Xauthority-*
rm -fr /root/.Xauthority-*


