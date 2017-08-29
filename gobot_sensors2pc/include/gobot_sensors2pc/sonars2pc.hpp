#ifndef SONAR2PC
#define SONAR2PC

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "pcl_ros/point_cloud.h"
#include "gobot_base/SonarMsg.h"
#include <XmlRpcValue.h>

void newSonarsInfo(const gobot_base::SonarMsg::ConstPtr& sonars);
bool initParams(void);

#endif