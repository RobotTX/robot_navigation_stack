#ifndef BATTERY_CONTROLLER
#define BATTERY_CONTROLLER

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/SetBattery.h>


/// Service to know if the robot is charging
bool isChargingService(gobot_msg_srv::IsCharging::Request &req, gobot_msg_srv::IsCharging::Response &res);

void newLeftBattery(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newRightBattery(const gazebo_msgs::ContactsState::ConstPtr& msg);

bool setBattery(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res);

#endif
