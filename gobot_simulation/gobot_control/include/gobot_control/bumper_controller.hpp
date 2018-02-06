#ifndef BUMPER_CONTROLLER
#define BUMPER_CONTROLLER

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "gazebo_msgs/ContactsState.h"
#include "gobot_msg_srv/BumperMsg.h"


void newBackLeftBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newBackLeftCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newBackRightBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newBackRightCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newFrontLeftBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newFrontLeftCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newFrontRightBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);
void newFrontRightCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);

#endif
