#ifndef PROXIMITY_CONTROLLER
#define PROXIMITY_CONTROLLER

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "gobot_base/ProximityMsg.h"

void newLeftSignal(const sensor_msgs::Range::ConstPtr& msg);
void newRightSignal(const sensor_msgs::Range::ConstPtr& msg);

#endif
