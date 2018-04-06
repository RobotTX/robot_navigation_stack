//ros headers
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <numeric> 
#include <serial/serial.h>
#include <thread>

#include <gobot_msg_srv/set_robot_class.h>


std::vector<uint8_t> REQUEST_DATA_CMD = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B};
std::vector<uint8_t> RESET_MCU_CMD = {0xD0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B};
std::vector<uint8_t> SHUT_DOWN_CMD = {0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x1B};


std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0);

bool displaySensorData(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res);

/// Send a command to reset the stm32
void resetStm(void);

/// Get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

void ledTimerCallback(const ros::TimerEvent&);

/// Check MCU when startup
bool checkSensors(void);

/// Read and publish all the sensors info
void publishSensors(void);

void initData(ros::NodeHandle &nh);

/// Initialize he serial connection
bool initSerial(void);

bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void displayBatteryLed(void);

bool setLed(int mode, const std::vector<std::string> &color);

bool setSound(int num,int time_on);

bool resetSTMSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void mySigintHandler(int sig);