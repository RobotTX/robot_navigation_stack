#ifndef SONAR2PC
#define SONAR2PC

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "gobot_msg_srv/SonarMsg.h"
#include <XmlRpcValue.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

double sonarRawToRange(double sonar_raw);
void sonarCallback(const gobot_msg_srv::SonarMsg::ConstPtr& sonars);
void initParams();

#endif