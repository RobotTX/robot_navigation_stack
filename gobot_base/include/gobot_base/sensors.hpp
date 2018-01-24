#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <thread>
#include "serial/serial.h"
#include <mutex>
#include <gobot_msg_srv/set_status_class.h>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/BumperMsg.h>
#include <gobot_msg_srv/IrMsg.h>
#include <gobot_msg_srv/ProximityMsg.h>
#include <gobot_msg_srv/SonarMsg.h>
#include <gobot_msg_srv/WeightMsg.h>
#include <gobot_msg_srv/CliffMsg.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/LedStrip.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetBattery.h>
#include <gobot_msg_srv/SetSpeeds.h>


std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0);

bool displaySensorData(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res);

/// Send a command to reset the stm32
void resetStm(void);

/// Service to know if the robot is charging
bool isChargingService(gobot_msg_srv::IsCharging::Request &req, gobot_msg_srv::IsCharging::Response &res);

bool PercentService(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res);

/// Get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

/// Read and publish all the sensors info
void publishSensors(void);

/// Initialize he serial connection
bool initSerial(void);

bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool setLedSrvCallback(gobot_msg_srv::LedStrip::Request &req, gobot_msg_srv::LedStrip::Response &res);

bool resetSTMSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void mySigintHandler(int sig);