#ifndef BATTERY_CHECK
#define BATTERY_CHECK

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <gobot_msg_srv/BatteryMsg.h>


void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo);

#endif
