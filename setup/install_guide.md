### To install Ubuntu ###

install Ubuntu 16.04 LTS using USB drive

#space allocation
swap=4GB,ext4/boot=2GB,ext4/=20GB,ext4/home=rest

###################################################

### To install ROS ###

install ros-kinetic-desktop (without gazebo)
http://wiki.ros.org/kinetic/Installation/Ubuntu

###################################################

### To Configure The System Files ###

#avoid password for sudo cmd
sudo visudo 
username ALL=(ALL) NOPASSWD: ALL

#allow fping
sudo apt install fping

#allow ssh username@ip
sudo apt-get install openssh-server openssh-client

#avoid sudo chmod for /dev/tty
sudo cp <rule file> /etc/udev/rules.d/>
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

#allow remote desktop
configure "Desktop sharing" to allow "Remmina Remote Desktop Client"

#log in without password
all settings->user accounts->automatic login

#robot startup
startup application->add "sudo sh /home/gtdollar/catkin_ws/src/gobot_navigation_stack/gobot_data/command/start_robot.sh"

#configure .bashrc
alias ls='ls -l --color'
alias cat_make='cd ~/catkin_ws/ && catkin_make && source devel/setup.bash && . ~/catkin_ws/devel/setup.bash'
alias shut='sudo shutdown -h now'
source /opt/ros/kinetic/setup.bash
source /home/username/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/username/catkin_ws/src

#screen split
settings >> appearance >> behavior
sudo apt install unity-tweak-tool

###################################################

### To Install Driver Packages ###

sudo apt install ros-kinetic-navigation
sudo apt install ros-kinetic-robot-pose-publisher
sudo apt install ros-kinetic-gmapping
sudo apt install ros-kinetic-openslam-gmapping
sudo apt install ros-kinetic-hector-nav-msgs
sudo apt install ros-kinetic-teb-local-planner
sudo apt install ros-kinetic-teleop-twist-keyboard
sudo apt install ros-kinetic-urg-node
sudo apt install ros-kinetic-hector-sensors-description (gazebo)
sudo apt install ros-kinetic-hector_gazebo-plugins (gazebo)

###################################################

### Installing ###

gobot_navigation_stack
Then catkin_make, then add the packages above to the src, then catkin_make again.
