<<<<<<< HEAD
# Gobot's catkin_ws sources

Contain all the custom packaged to run Gobot : 

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
=======
# robot
>>>>>>> a48dabbed7fd3dc124c6983ffcebecdfaab42b81
