#ifndef BATTERY_CHECK
#define BATTERY_CHECK

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/SetBattery.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/IsCharging.h>

bool testAutoDocking(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res);

void timerCallback(const ros::TimerEvent&);

#endif
