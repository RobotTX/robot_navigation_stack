#ifndef SONAR2PC
#define SONAR2PC

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "pcl_ros/point_cloud.h"
#include "gobot_msg_srv/SonarMsg.h"
#include <XmlRpcValue.h>

void sonarToCloud(int sonarData,pcl::PointCloud<pcl::PointXYZ> &cloudData);
void newSonarsInfo(const gobot_msg_srv::SonarMsg::ConstPtr& sonars);
bool initParams(void);

#endif