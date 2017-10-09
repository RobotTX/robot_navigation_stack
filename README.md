# Gobot's catkin_ws sources

Contain all the custom packaged to run Gobot

## Getting Started

### Prerequisites

Packages you'll need to install in order to make everything works :
* navigation
* robot_pose_publisher
* gmapping (actually just need open_slam)
* hector_nav_msgs
* teb_local_planner
* hector_sensors_description (gazebo)
* hector_gazebo-plugins (gazebo)
* telop_twist_keyboard (if you want to make the robot with your keyboard)

To install this packages : 

```
sudo apt install ros-kinetic-navigation
sudo apt install ros-kinetic-robot-pose-publisher
sudo apt install ros-kinetic-gmapping
sudo apt install ros-kinetic-openslam-gmapping
sudo apt install ros-kinetic-hector-nav-msgs
sudo apt install ros-kinetic-teb-local-planner
sudo apt install ros-kinetic-telop-twist-keyboard
sudo apt install ros-kinetic-urg-node
sudo apt install ros-kinetic-hector-sensors-description (gazebo)
```
To be installed:

'''
sudo visudo (avoid password for sudo cmd)

*Log in without password

sudo apt install fping

bashrc:
alias ls='ls -l --color'
alias cat_make='cd ~/catkin_ws/ && catkin_make && source devel/setup.bash && . ~/catkin_ws/devel/setup.bash'
alias shut='sudo shutdown -h now'
source /opt/ros/kinetic/setup.bash
source /home/username/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/username/catkin_ws/src

udev rules (avoid sudo chmod for /dev/tty)
sudo cp <rule file> /etc/udev/rules.d/>
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

sudo apt-get install openssh-server openssh-client

settings >> appearance >> behavior
sudo apt install unity-tweak-tool
'''

### Installing

Then catkin_make, then add the packages above to the src, then catkin_make again.

The packages above needs gobot_base to be compiled first and the dependencies in package.xml and CMakeLists.txt are not working correctly yet (need investigation).

# Description of the packages :

## gmapping

Package to create a map, based on the ros gmapping package, modified to add the robot's footprint to the published map.

## gobot_base

Contains the basic nodes to make the robot move, get sensors data and the laser data.

### sensors

Node to get the sensors data and publish them on their respective topics :
* bumpers => bumpers_topic,
* infrared sensors => ir_topic,
* proximity sensors => proximity_topic,
* sonars => sonar_topic,
* load cells => weight_topic,
* battery  => battery_topic,
* cliff sensors => cliff_topic.

### wheels

Communicate with the stm32 to get the encoders data of the wheels and to send the velocity commands.

### twist

Subscrive to the cmd_vel topic to translate the twist messages into actual usable velocity commands to send to the wheels

### odom

Publish the odometry of the robot => the position of the robot based on the wheels velocity

## gobot_navigation

Contains the launch file + param files for amcl, move_base, gmapping and hector_exploration

## gobot_sensors2pc

Contains 2 nodes to translate the bumpers and sonars data into pointclouds to be used by the costmap

## gobot_simulation

Contains everything to simulate the robot with gazebo

### gobot_control

Publish the sensors data to their respective topic (see above)

### gobot_description

The urdf files which describe the robot

### gobot_gazebo

Contains the launch file + the world file

## gobot_software

Contains all the nodes to communicate with the Gobot application on another computer

## hector_exploration_node

Create a wrapper around the hector_exploration_planner
The package is supposed to be used with the hector navigation stack so it also contains a node, move_base_controller, to translate the data from the planner into goal that move_base can use

## hector_exploration_planner

Package to explore the map while scanning, only contains classes to be used by another node => hector_exploration_node

## serial

Contains classes to easily communicate through serials connections with a pyton-like interface

## urg_node

Package to receive the laser data
Modified to make it works like the previous package, hokuyo_node, so that when data are received with a special status because the space in front of the robot is empty, we receive +inf values instead of NaN values.
