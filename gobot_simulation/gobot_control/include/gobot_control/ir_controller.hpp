#ifndef IR_CONTROLLER
#define IR_CONTROLLER

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "gobot_base/IrMsg.h"

void newRearSignal_1(const sensor_msgs::Image::ConstPtr& msg);
void newRearSignal_2(const sensor_msgs::Image::ConstPtr& msg);
void newLeftSignal_1(const sensor_msgs::Image::ConstPtr& msg);
void newLeftSignal_2(const sensor_msgs::Image::ConstPtr& msg);
void newRightSignal_1(const sensor_msgs::Image::ConstPtr& msg);
void newRightSignal_2(const sensor_msgs::Image::ConstPtr& msg);

int16_t convertSignal(const uint8_t red, const uint8_t green, const uint8_t blue);
int16_t multiSignalToSolo(const int16_t sig1, const int16_t sig2);

#endif
