#ifndef SCAN_FUNCTION
#define SCAN_FUNCTION

#include <ros/ros.h>
#include <signal.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>

#include <gobot_msg_srv/robot_move_class.h>
#include <gobot_msg_srv/GetGobotStatus.h>

//****************************** CALLBACK ******************************

//****************************** SERVICE ******************************
bool startExplorationSrv(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res);

bool stopExplorationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

//****************************** FUNCTIONS ******************************
void doExploration();

void backToStart();

void stopExploration();

void initData();

#endif