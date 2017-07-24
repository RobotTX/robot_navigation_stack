#ifndef BATTERY_CONTROLLER
#define BATTERY_CONTROLLER

#include "ros/ros.h"
#include "gazebo_msgs/ContactsState.h"
#include "gobot_base/BatteryMsg.h"
#include "gobot_control/SetBattery.h"


void newLeftBattery(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newRightBattery(const gazebo_msgs::ContactsState::ConstPtr& msg);

bool setBattery(gobot_control::SetBattery::Request &req, gobot_control::SetBattery::Response &res);

#endif
