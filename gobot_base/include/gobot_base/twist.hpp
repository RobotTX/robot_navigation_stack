#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <gobot_msg_srv/BumperMsg.h>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/SetInt.h>
#include "gobot_msg_srv/CliffMsg.h"
#include <chrono>
#include <thread>

bool continueRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool pauseRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Call the service to set the speed of the wheels
bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);

/// Callback of the subscriber on bumper infoso that if we bump into something, the robot top
void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);

void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff);

/// Callback of the subscriber on velocity commands
void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist);