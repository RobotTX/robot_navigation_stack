#ifndef AUTO_DOCKING
#define AUTO_DOCKING

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <string>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/smart_ptr.hpp>
#include <sonar/GetBatteryInfo.h>
#include <sonar/GetIrSignal.h>
#include <sonar/GetShortSignal.h>
#include <sonar/GetBumpers.h>
#include <wheel/SetSpeeds.h>
#include <ctime>
#include <chrono>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool startDocking(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void stopDocking(void);
void initCurrentGoal(void);
void getRobotPos(const geometry_msgs::Pose::ConstPtr& currentGoal);
void goalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray);
bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
bool pidControl(void);
void findChargingStation(void);


#endif
