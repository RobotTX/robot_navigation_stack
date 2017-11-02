#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <hector_exploration_node/Exploration.h>

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg);

void mySigintHandler(int sig);