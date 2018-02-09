#include <ros/ros.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <signal.h>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/GetEncoders.h>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/GetIntArray.h>
#include <serial/serial.h>


/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0);

/// get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

/// Initialize the serial connection
bool initSerial(void);

bool getSpeeds(gobot_msg_srv::GetIntArray::Request &req, gobot_msg_srv::GetIntArray::Response &res);

/// Set the speed, 0 (full reverse)  128 (stop)   255 (full forward)
bool setSpeeds(gobot_msg_srv::SetSpeeds::Request &req, gobot_msg_srv::SetSpeeds::Response &res);

/// Get the encoders position
bool getEncoders(gobot_msg_srv::GetEncoders::Request &req, gobot_msg_srv::GetEncoders::Response &res);

/// Set the encoders to 0
bool resetEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool initialMotor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Just to test, we get the encoders and print them
bool testEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void mySigintHandler(int sig);