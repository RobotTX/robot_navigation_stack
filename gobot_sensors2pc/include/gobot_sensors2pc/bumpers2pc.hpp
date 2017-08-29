#ifndef BUMPERS2PC
#define BUMPERS2PC

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "pcl_ros/point_cloud.h"
#include "gobot_base/BumperMsg.h"
#include <XmlRpcValue.h>

void newBumpersInfo(const gobot_base::BumperMsg::ConstPtr& bumpers);
double xToY(const double x, const std::vector<std::vector<double>>& footprint, const bool front_bumper);
void initBumperPC(const std::vector<std::vector<double>>& footprint, const std::vector<std::vector<double>>& bumpers_description);
bool initParams(void);
bool vectorFromXMLRPC(XmlRpc::XmlRpcValue list, std::vector<std::vector<double>>& vector);

#endif