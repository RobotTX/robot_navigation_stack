### To install Ubuntu ###

install Ubuntu 16.04 LTS using USB drive

#space allocation
swap=4GB,ext4/boot=2GB,ext4/=20GB,ext4/home=rest

###################################################

### To install ROS ###

install ros-kinetic-desktop (without gazebo)
http://wiki.ros.org/kinetic/Installation/Ubuntu

###################################################

### To Configure The System ###

#avoid password for sudo cmd
sudo visudo 
[username] ALL=(ALL) NOPASSWD: ALL

#avoid sudo chmod for /dev/tty
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)
sudo cp [rules_file] /etc/udev/rules.d/>
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

#log in without password
all settings->user accounts->automatic login

#robot startup
startup application->add "sudo sh /home/[username]/catkin_ws/src/gobot_navigation_stack/gobot_data/command/start_robot.sh"

#configure .bashrc
alias ls='sudo apt install gimpls -l --color'
alias cat_make='cd ~/catkin_ws/ && catkin_make && source devel/setup.bash && . ~/catkin_ws/devel/setup.bash'
alias shut='sudo shutdown -h now'
source /opt/ros/kinetic/setup.bash
source /home/[username]/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/[username]/catkin_ws/src

#allow fping
sudo apt install fping

#allow ssh [username]@ip
sudo apt install openssh-server openssh-client

#remote desktop control
configure "Desktop sharing" to allow "Remmina Remote Desktop Client"
remmina remote desktop client
protocol: VNC

#network manager
sudo apt install network-manager

#create hotspot
nmcli d wifi hotspot ssid "GTrobotwifi" password "gt123456"

### Optional Configuration ###

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

### To Install Driver Packages ###

sudo sh /home/[username]/catkin_ws/src/gobot_navigation_stack/setup/driver_package.sh

sudo apt install ros-kinetic-navigation
sudo apt install ros-kinetic-robot-pose-publisher
sudo apt install ros-kinetic-gmapping
sudo apt install ros-kinetic-openslam-gmapping
sudo apt install ros-kinetic-hector-nav-msgs
sudo apt install ros-kinetic-urg-node
sudo apt install ros-kinetic-hector-sensors-description (gazebo)
sudo apt install ros-kinetic-hector_gazebo-plugins (gazebo)

###################################################

### Installing ###

gobot_navigation_stack
Then catkin_make, then add the packages above to the src, then catkin_make again.

#footprint & other parameters
gobot_base.launch   wheel_separation    0.45
                    wheel_radius        0.0725
                    ticks_per_rotation  *819.2
*costmap.yaml/gmapping.yaml/teb_local.yaml/bumpers2pc.yaml
[[0,0.26],[0.1,0.255],[0.178,0.25],[0.222,0.225],[0.245,0.112],[0.25,0],[0.245,-0.112],[0.222,-0.225],[0.178,-0.25],[0.1,-0.255],[0,-0.26],[-0.1,-0.255],[-0.178,-0.25],[-0.222,-0.225],[-0.245,-0.112],[-0.25,0],[-0.245,0.112],[-0.222,0.225],[-0.178,0.25],[-0.1,0.255]]

*costmap_common.yaml/teb_local_planner.yaml
sqrt(0.25^2+0.26^2) = 0.36
inflation_radius = 0.4/0.45/0.5
