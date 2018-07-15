#ifndef DOCKING_FUNCTION
#define DOCKING_FUNCTION

#include <ros/ros.h>
#include <ctime>
#include <chrono>
#include <signal.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <gobot_msg_srv/get_robot_class.h>
#include <gobot_msg_srv/robot_move_class.h>

//****************************** CALLBACK ******************************
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo);

void newIrSignal(const gobot_msg_srv::IrMsg::ConstPtr& irSignal);

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

//****************************** SERVICE ******************************
bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

//****************************** FUNCTIONS ******************************
bool startDocking();

void findChargingStation();

void finishedDocking(bool move_forward = true);

void failedDocking(std::string reason="");

void resetDockingParams(void);

void startDockingParams(void);

#endif