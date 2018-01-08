#ifndef BUMPERS2PC
#define BUMPERS2PC

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "pcl_ros/point_cloud.h"
#include "gobot_msg_srv/BumperMsg.h"

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);
bool initParams(void);
#endif