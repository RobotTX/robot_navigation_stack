#ifndef AUTO_DOCKING
#define AUTO_DOCKING

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <chrono>
#include <thread>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/smart_ptr.hpp>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>
#include <gobot_msg_srv/robot_msgs.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool startDocking(void);
void findChargingStation(void);
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);
void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo);
void newIrSignal(const gobot_msg_srv::IrMsg::ConstPtr& irSignal);
void newProximityInfo(const gobot_msg_srv::ProximityMsg::ConstPtr& proximitySignal);
void finishedDocking(bool move_forward = true);
bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
void resetDockingParams(void);
void startDockingParams(void);
#endif