#ifndef PLAY_PATH
#define PLAY_PATH

#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <boost/bind.hpp>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <cmath>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/SetStage.h>
#include <gobot_msg_srv/GetStage.h>
#include <gobot_msg_srv/SetPath.h>
#include <gobot_msg_srv/GetPath.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/SetInt.h>
#include <thread>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point {
	double x;
	double y;
    // time that you have to spend waiting before heading to this point
	double waitingTime;
    // whether or not this point is the charging point of the robot
    bool isHome;

    double yaw;
};

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool skipPathService(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res);
void goNextPoint(void);
bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void goalReached(void);
void setStageInFile(const int _stage);
void setGobotStatus(int status,std::string text);
void getButtonCallback(const std_msgs::Int8::ConstPtr& msg);
void checkGoalDelay(void);
#endif