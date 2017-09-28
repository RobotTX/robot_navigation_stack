#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Int8.h>
#include <gobot_msg_srv/LedStrip.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <gobot_msg_srv/BumperMsg.h>
#include <gobot_msg_srv/BatteryMsg.h>


void setLedPermanent(std::vector<uint8_t> &color);

void setLedRunning(std::vector<uint8_t> &color);

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

void goalGetCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

void initialPoseCallback(const std_msgs::Int8::ConstPtr& msg);

void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& msg);

void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg);