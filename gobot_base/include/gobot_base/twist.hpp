//ros headers
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <numeric> 
#include <thread>

#include <gobot_msg_srv/set_robot_class.h>

bool continueRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool pauseRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Callback of the subscriber on bumper infoso that if we bump into something, the robot top
void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);

bool cliffOutRange(double CliffData);

void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff);

void lostCallback(const std_msgs::Int8::ConstPtr& msg);

void joyConnectionCallback(const std_msgs::Int8::ConstPtr& data);

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

/// Callback of the subscriber on velocity commands
void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist);

void statusCallback(const std_msgs::Int8::ConstPtr& msg);
    
void cmdToMotorSpeed(double cmd_linear, double cmd_angular);

void initParams(ros::NodeHandle &nh);

void checkCollisionTimer(const ros::TimerEvent&);
