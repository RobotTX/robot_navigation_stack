#ifndef SONAR_CONTROLLER
#define SONAR_CONTROLLER

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "gobot_msg_srv/SonarMsg.h"

void newRearSignal(const sensor_msgs::Range::ConstPtr& msg);
void newLeftSignal(const sensor_msgs::Range::ConstPtr& msg);
void newRightSignal(const sensor_msgs::Range::ConstPtr& msg);
void newFrontLeftSignal(const sensor_msgs::Range::ConstPtr& msg);
void newFrontRightSignal(const sensor_msgs::Range::ConstPtr& msg);

#endif
