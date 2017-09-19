#ifndef BATTERY_CHECK
#define BATTERY_CHECK

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <gobot_base/BatteryMsg.h>

void newBatteryInfo(const gobot_base::BatteryMsg::ConstPtr& batteryInfo);

#endif
