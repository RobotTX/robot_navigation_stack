#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "serial/serial.h"
#include <thread>

/// Send a command to reset the stm32
void resetStm(void);

/// Service to know if the robot is charging
bool isChargingService(gobot_msg_srv::IsCharging::Request &req, gobot_msg_srv::IsCharging::Response &res);

/// Get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

/// Read and publish all the sensors info
void publishSensors(void);

/// Initialize he serial connection
bool initSerial(void);

bool setLedCallback(gobot_msg_srv::LedStrip::Request &req, gobot_msg_srv::LedStrip::Response &res);