#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <gobot_msg_srv/GoalStatus.h>
#include <gobot_msg_srv/GetGobotStatus.h>

void initialPoseResultCallback(const std_msgs::Int8::ConstPtr& msg);

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg);

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

void goalGetCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

bool goalStatusSrvCallback(gobot_msg_srv::GoalStatus::Request &req, gobot_msg_srv::GoalStatus::Response &res);

void mySigintHandler(int sig);