### To install Ubuntu ###

install Ubuntu 16.04 LTS using USB drive

#space allocation
swap=4GB,ext4/boot=2GB,ext4/=20GB,ext4/home=rest

###################################################

### To install ROS ###

install ros-kinetic-desktop (without gazebo)
http://wiki.ros.org/kinetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop
sudo rosdep init
rosdep update
*sudo rosdep fix-permissions (if roscore can not run)
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

#create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

#configure .bashrc
alias cat_make='cd ~/catkin_ws/ && catkin_make && source devel/setup.bash && . ~/catkin_ws/devel/setup.bash'
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src


#install Driver Packages
sudo sh ~/catkin_ws/src/gobot_navigation_stack/setup/install_script.sh
#*
sudo apt install ros-kinetic-navigation
sudo apt install ros-kinetic-robot-pose-publisher
sudo apt install ros-kinetic-teb-local-planner
sudo apt install ros-kinetic-gmapping
sudo apt install ros-kinetic-openslam-gmapping
sudo apt install ros-kinetic-hector-nav-msgs
sudo apt install ros-kinetic-urg-node
sudo apt install ros-kinetic-hector-sensors-description (gazebo)
sudo apt install ros-kinetic-hector_gazebo-plugins (gazebo)
sudo apt-get install libusb-dev
sudo apt-get install libspnav-dev
###################################################

#use usb-connected joystick#
#*
sudo apt-get install libusb-dev
sudo apt-get install libspnav-dev

### Installing Gobot System###
gobot_navigation_stack
Then catkin_make, then add the packages above to the src, then catkin_make again.

#permission for .cfg file
sudo chmod -R +x ~/catkin_ws/src/gobot_navigation_stack

#footprint & other parameters
robot_sensors_param.launch      wheel_separation    middle:0.351  outside:0.402
                                wheel_radius        0.0725 -> 0.075
                                ticks_per_rotation  *819.2 -> 980
* footprint
costmap.yaml/gmapping.yaml/teb_local.yaml
[[0.250,0],[0.245,-0.165], [0.225,-0.220], [0.165,-0.240],
[0,-0.245],[-0.165,-0.240],[-0.225,-0.220],[-0.245,-0.165],
[-0.250,0],[-0.245,0.165], [-0.225,0.220], [-0.165,0.240],
[0,0.245], [0.165,0.240],  [0.225,0.220],  [0.245,0.165]]

* TF
-sonar z=0.3-0.4 front_right:0.243,0.12; front_left:0.243,-0.12; back_right:-0.246,0.112; back_left:-0.246,-0.112
-laser z=0.2-0.3 200,0
-cliff z=0-0.1 front_right:0.215,0.18; front_left:0.215,-0.18; back_right:-0.215,0.18; back_left:-0.215,-0.18
-bumpers z=0.1-0.2 [0.125,0.225] [0.01,0.125] [-0.125,-0.01] [-0.225,-0.125]

*costmap_common.yaml/teb_local_planner.yaml
sqrt(0.255^2+0.25^2) = 0.357  
inflation_radius = 0.45     0.4/0.45/0.5  
s
* laser
amcl.yaml
costmap_common_params.yaml
gmapping_params.yaml

###################################################

### To Configure The System ###
#avoid password for sudo cmd
sudo visudo 
username ALL=(ALL) NOPASSWD: ALL

#avoid sudo chmod for /dev/tty
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) | grep KERNELS | grep "-" | grep ":" | cut -d '"' -f2
create_udevrules_script.sh

#log in without password
all settings->user accounts->automatic login

#robot startup
* 1.startup application->add
sudo sh /home/username/catkin_ws/src/gobot_navigation_stack/gobot_data/command/start_robot.sh
* 2./etc/rc.local （failed)
username="gtdollar"
cd /home/$username/catkin_ws/src/gobot_navigation_stack/gobot_data/command
./start_robot.sh $username
before exit0
    

#allow fping
#*
sudo apt install fping

#allow ssh [username]@ip
#*
sudo apt install openssh-server openssh-client

#network manager
#*
sudo apt install network-manager

#permission denied when TAB after roslaunch
sudo umount /home/useraccount/.gvfs
sudo rm -rf .gvfs/

### Optional Configuration ###

#remote desktop control
configure "Desktop sharing" to allow "Remmina Remote Desktop Client"
remmina remote desktop client
protocol: VNC

#create hotspot
nmcli d wifi hotspot ssid "GTrobotwifi" password "gt123456"

#screen split
settings >> appearance >> behavior
sudo apt install unity-tweak-tool

#file transfer
sudo apt install filezilla
protocol: SFTP

#Picture editor GIMP
sudo apt install gimp

#QT 5.8
sudo ./qt.run

#roboware studio
sudo dpkg -i robotware.deb

###################################################
