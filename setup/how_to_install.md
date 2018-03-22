### To install Ubuntu ###

install Ubuntu 16.04 LTS using USB drive

#space allocation
swap=2GB (L),ext4/boot=500MB (L),ext4/=15GB (P),ext4/home=10GB (L)

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
export ROS_MASTER_URI=http://localhost:11311

#install Driver Packages
sudo sh ~/catkin_ws/src/gobot_navigation_stack/setup/install_script.sh

### Installing Gobot System ###

#permission for .cfg file
sudo chmod -R +x ~/catkin_ws/src/gobot_navigation_stack

gobot_navigation_stack
Then catkin_make, then add the packages above to the src, then catkin_make again.

### To Configure The System ###
#avoid password for sudo cmd
sudo visudo 
username ALL=(ALL) NOPASSWD: ALL

#avoid sudo chmod for /dev/tty
sudo sh ~/catkin_ws/src/gobot_navigation_stack/setup/create_udevrules_script.sh

#log in without password
all settings->user accounts->automatic login

#robot startup
startup application->add
sudo sh /home/username/catkin_ws/src/gobot_navigation_stack/gobot_data/command/start_robot.sh

#tts
refer to tts example/tts_guide.md for installment