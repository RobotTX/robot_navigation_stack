#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/SetString.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/IsCharging.h>

#define PI 3.1415926

#define COV_XY_T 0.2
#define COV_YAW_T 0.1

#define START_STAGE 0
#define CHARGING_STAGE 1
#define ROSPARAM_POSE_STAGE 2
#define LAST_POSE_STAGE 3
#define GLOBAL_POSE_STAGE 4
#define COMPLETE_STAGE 5

#define START_FOUND -1
#define NOT_FOUND 0
#define FOUND 1
#define CANCEL_FOUND 2


bool evaluatePose(int type);

bool rotateFindPose(double rot_v,double rot_t);

void findPoseResult(int status);

void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w,const double cov1,const double cov2);

void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

bool initializePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool globalizePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopGlobalizePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool goHomeSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

