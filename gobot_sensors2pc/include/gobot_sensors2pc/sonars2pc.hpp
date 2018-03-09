#ifndef SONAR2PC
#define SONAR2PC

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "pcl_ros/point_cloud.h"
#include "gobot_msg_srv/SonarMsg.h"
#include <XmlRpcValue.h>
#include <gobot_msg_srv/MotorSpeedMsg.h>

void sonarFrontToCloud(double sonarR,double sonarL,pcl::PointCloud<pcl::PointXYZ> &cloudR,pcl::PointCloud<pcl::PointXYZ> &cloudL, double y, double factor);
void sonarBackToCloud(double sonarR,double sonarL,pcl::PointCloud<pcl::PointXYZ> &cloudR,pcl::PointCloud<pcl::PointXYZ> &cloudL, double y, double factor);
void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed);
void newSonarsInfo(const gobot_msg_srv::SonarMsg::ConstPtr& sonars);
bool initParams(void);

#endif