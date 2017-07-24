#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <gobot_base/BumperMsg.h>
#include <gobot_base/SetSpeeds.h>
#include <chrono>
#include <thread>

/// Call the service to set the speed of the wheels
bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);

/// Callback of the subscriber on bumper infoso that if we bump into something, the robot top
void newBumpersInfo(const gobot_base::BumperMsg::ConstPtr& bumpers);

/// Callback of the subscriber on velocity commands
void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist);