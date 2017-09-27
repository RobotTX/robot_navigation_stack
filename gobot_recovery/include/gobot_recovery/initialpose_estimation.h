#include <ros/ros.h>
#include <std_msgs/Int8.h>
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
#include <signal.h>

#define COV_XY_T 0.2
#define COV_YAW_T 0.1

#define START_STATE 0
#define CHARGING_STATE 1
#define ROSPARAM_POSE_STATE 2
#define LAST_POSE_STATE 3
#define CHARGING_POSE_STATE 4
#define GLOBAL_POSE_STATE 5
#define COMPLETE_STATE 6

void checkGoalStatus(void);

bool rotateFindPose(double rot_v,double rot_t);

void GlobalLocalization(void);

void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w,const double cov);

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

bool initializePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool globalizePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopGlobalizePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void getPose(std::string file_name,int type);

