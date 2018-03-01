#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "pcl_ros/point_cloud.h"
#include "gobot_msg_srv/CliffMsg.h"
#include "gobot_msg_srv/SetSpeeds.h"
#include <gobot_msg_srv/GetIntArray.h>
#include <gobot_msg_srv/MotorSpeedMsg.h>
#include <thread>

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff);
bool cliffToCloud(double CliffData,pcl::PointCloud<pcl::PointXYZ> &cloudData, bool cliff_on);
bool initParams(void);