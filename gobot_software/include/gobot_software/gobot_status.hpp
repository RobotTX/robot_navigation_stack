#include <ros/ros.h>
#include <string>
#include <thread>
#include <mutex>
#include <fstream>
#include <iostream>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/SetDockStatus.h>
#include <gobot_msg_srv/GetDockStatus.h>
#include <gobot_msg_srv/SetStage.h>
#include <gobot_msg_srv/GetStage.h>
#include <gobot_msg_srv/SetPath.h>
#include <gobot_msg_srv/GetPath.h>
#include <gobot_msg_srv/SetString.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/IsCharging.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>

bool disconnectedSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool setWifiSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);
bool getWifiSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res);

bool setHomeSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);
bool getHomeSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res);

bool setBatterySrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);
bool getBatterySrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res);

bool setSpeedSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);
bool getSpeedSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res);

bool setNameSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);
bool getNameSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res);

bool setPathSrvCallback(gobot_msg_srv::SetPath::Request &req, gobot_msg_srv::SetPath::Response &res);
bool getPathSrvCallback(gobot_msg_srv::GetPath::Request &req, gobot_msg_srv::GetPath::Response &res);

bool setLoopSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res);
bool getLoopSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res);

bool setStageSrvCallback(gobot_msg_srv::SetStage::Request &req, gobot_msg_srv::SetStage::Response &res);
bool getStageSrvCallback(gobot_msg_srv::GetStage::Request &req, gobot_msg_srv::GetStage::Response &res);

bool setDockStatusSrvCallback(gobot_msg_srv::SetDockStatus::Request &req, gobot_msg_srv::SetDockStatus::Response &res);
bool getDockStatusSrvCallback(gobot_msg_srv::GetDockStatus::Request &req, gobot_msg_srv::GetDockStatus::Response &res);

bool setGobotStatusSrvCallback(gobot_msg_srv::SetGobotStatus::Request &req, gobot_msg_srv::SetGobotStatus::Response &res);
bool getGobotStatusSrvCallback(gobot_msg_srv::GetGobotStatus::Request &req, gobot_msg_srv::GetGobotStatus::Response &res);

bool getMuteSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res);
bool setLoopSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res);

bool isChargingService(gobot_msg_srv::IsCharging::Request &req, gobot_msg_srv::IsCharging::Response &res);

bool PercentService(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res);

void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg);

void initialData(void);