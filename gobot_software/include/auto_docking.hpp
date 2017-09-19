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
#include <gobot_base/SetSpeeds.h>
#include <gobot_base/BatteryMsg.h>
#include <gobot_base/BumperMsg.h>
#include <gobot_base/IrMsg.h>
#include <gobot_base/ProximityMsg.h>
#include <gobot_software/SetDockStatus.h>
#include <ctime>
#include <chrono>
#include <thread>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool startDocking(void);
bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void stopDocking(void);
void newRobotPos(const geometry_msgs::Pose::ConstPtr& currentGoal);
void goalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray);
bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
bool pidControl(void);
void failedDocking(const int status);
void checkBumpers(void);
void findChargingStation(void);
void newBatteryInfo(const gobot_base::BatteryMsg::ConstPtr& batteryInfo);
void newBumpersInfo(const gobot_base::BumperMsg::ConstPtr& bumpers);
void alignWithCS(void);
void newIrSignal(const gobot_base::IrMsg::ConstPtr& irSignal);
void newProximityInfo(const gobot_base::ProximityMsg::ConstPtr& irSignal);
void finishedDocking(const int16_t status);

#endif
