#include <ros/ros.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <std_srvs/Empty.h>
#include <gobot_base/GetEncoders.h>
#include <gobot_base/SetSpeeds.h>
#include "serial/serial.h"


/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0);

/// get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

/// Initialize the serial connection
bool initSerial(void);

/// Set the speed, 0 (full reverse)  128 (stop)   255 (full forward)
bool setSpeeds(gobot_base::SetSpeeds::Request &req, gobot_base::SetSpeeds::Response &res);

/// Get the encoders position
bool getEncoders(gobot_base::GetEncoders::Request &req, gobot_base::GetEncoders::Response &res);

/// Set the encoders to 0
bool resetEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Just to test, we get the encoders and print them
bool testEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Get the encoders and print the difference between the new and previous encoder every 1 sec
bool testEncoders2(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);