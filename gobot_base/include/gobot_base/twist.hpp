#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/robot_msgs.h>
#include <gobot_msg_srv/GetIntArray.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool continueRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool pauseRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Callback of the subscriber on bumper infoso that if we bump into something, the robot top
void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);

void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff);

void lostCallback(const std_msgs::Int8::ConstPtr& msg);

void joyConnectionCallback(const std_msgs::Int8::ConstPtr& data);


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

/// Callback of the subscriber on velocity commands
void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist);

void initParams(ros::NodeHandle &nh);
