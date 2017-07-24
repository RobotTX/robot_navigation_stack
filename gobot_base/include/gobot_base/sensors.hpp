#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <gobot_base/SetSpeeds.h>
#include <gobot_base/SetSpeeds.h>
#include <gobot_base/BatteryMsg.h>
#include <gobot_base/BumperMsg.h>
#include <gobot_base/IrMsg.h>
#include <gobot_base/ProximityMsg.h>
#include <gobot_base/SonarMsg.h>
#include <gobot_base/WeightMsg.h>
#include <gobot_base/CliffMsg.h>
#include "serial/serial.h"


/// Get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

/// Read and publish all the sensors info
void publishSensors(void);

/// Initialize he serial connection
bool initSerial(void);