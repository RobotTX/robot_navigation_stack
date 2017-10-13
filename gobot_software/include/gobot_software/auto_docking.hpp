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
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/BumperMsg.h>
#include <gobot_msg_srv/IrMsg.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/ProximityMsg.h>
#include <gobot_msg_srv/SetDockStatus.h>
#include <gobot_msg_srv/GoalStatus.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <ctime>
#include <chrono>
#include <thread>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void setGobotStatus(int status,std::string text);
bool startDocking(void);
void newRobotPos(const geometry_msgs::Pose::ConstPtr& robotPos);
void goalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray);
void findChargingStation(void);
bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers);
void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo);
void newIrSignal(const gobot_msg_srv::IrMsg::ConstPtr& irSignal);
void alignWithCS(void);
void newProximityInfo(const gobot_msg_srv::ProximityMsg::ConstPtr& proximitySignal);
void finishedDocking(const int16_t status);
void failedDocking(const int status);
void stopDocking(void);
bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

#endif