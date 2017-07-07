#ifndef PLAY_PATH
#define PLAY_PATH

#include "ros/ros.h"
#include <fstream>
#include <vector>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include <boost/bind.hpp>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point {
	double x;
	double y;
    // time that you have to spend waiting before heading to this point
	double waitingTime;
    // whether or not this point is the charging point of the robot
    bool isHome;
};

void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg);
void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& _status);
bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void goNextPoint();
bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void goToPoint(const Point& point);
void goalReached();
void setStageInFile(const int _stage);

#endif