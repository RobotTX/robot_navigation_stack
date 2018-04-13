#ifndef PLAY_PATH
#define PLAY_PATH

#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <signal.h>
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
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>
#include <thread>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point {
	double x;
	double y;
    // time that you have to spend waiting before heading to this point
	double waitingTime;
    // whether or not this point is the charging point of the robot
    bool isHome;
    double yaw;
    std::string text;
    double delayText;
};

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

void goalReached();

void checkGoalDelay();

void textToSpeech(std::string text, double delay);

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg);

void goNextPoint(const Point _point);

bool savePointService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool playPointService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool updatePathService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool goDockAfterPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool interruptDelayService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool skipPathService(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res);

void initData();

void mySigintHandler(int sig);
#endif