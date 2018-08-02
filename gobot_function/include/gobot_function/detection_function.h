#ifndef SCAN_FUNCTION
#define SCAN_FUNCTION

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <gobot_msg_srv/robot_move_class.h>

//****************************** CALLBACK ******************************
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

void magnetCb(const std_msgs::Int8::ConstPtr& msg);

void bumpersCb(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);

void alignmentCb(const std_msgs::Int16::ConstPtr& msg);

//****************************** SERVICE ******************************
bool findObjectCb(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool startDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

//****************************** FUNCTIONS ******************************
void startAlignObject();

void stopDetectionFunc(std::string result, std::string status_text="STOP_TRACKING");

bool roughAlignment();

void timerCallback(const ros::TimerEvent&);
    
void mySigintHandler(int sig);

#endif