#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <gobot_msg_srv/IsCharging.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <thread>

void checkGoalStatus(void);

bool rotateFindPose(double rot_v,double rot_t);

bool GlobalLocalization(void);

void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w);

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

bool checkInitPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool globalizePoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopGlobalizePoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void getLastPose(std::string data);

