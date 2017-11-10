#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/LedStrip.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/BumperMsg.h>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/SetInt.h>


void setLedPermanent(std::vector<uint8_t> &color);
void setSound(int num,int time_on, int time_off=0);

void setLedRunning(std::vector<uint8_t> &color);

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

void goalGetCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

void goalCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);

void initialPoseResultCallback(const std_msgs::Int8::ConstPtr& msg);

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& msg);

void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg);

void explorationCallback(const std_msgs::Int8::ConstPtr& msg);

void lostCallback(const std_msgs::Int8::ConstPtr& msg);
 
bool showBatteryLedsrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void batteryLed(void);

void timerCallback(const ros::TimerEvent&);

