#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/SetIntArray.h>


void saveMap(void);

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg);

bool saveMapSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void mySigintHandler(int sig);