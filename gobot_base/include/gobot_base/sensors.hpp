#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <thread>
#include "serial/serial.h"
#include <mutex>
#include <gobot_msg_srv/robot_msgs.h>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>



std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0);

bool displaySensorData(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res);

/// Send a command to reset the stm32
void resetStm(void);

/// Get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

void ledTimerCallback(const ros::TimerEvent&);

/// Read and publish all the sensors info
void publishSensors(void);

/// Initialize he serial connection
bool initSerial(void);

bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool setLedSrvCallback(gobot_msg_srv::LedStrip::Request &req, gobot_msg_srv::LedStrip::Response &res);

void displayBatteryLed(void);

bool setLed(int mode, const std::vector<std::string> &color);

bool setSound(int num,int time_on);

bool resetSTMSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void mySigintHandler(int sig);