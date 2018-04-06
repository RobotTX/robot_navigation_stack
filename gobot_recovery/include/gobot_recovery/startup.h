#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <gobot_msg_srv/get_robot_class.h>

void initialPoseResultCallback(const std_msgs::Int8::ConstPtr& msg);

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg);

void mySigintHandler(int sig);