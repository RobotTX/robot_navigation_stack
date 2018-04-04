#include <ros/ros.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <signal.h>
#include <numeric> 
#include <std_srvs/Empty.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gobot_msg_srv/OdomTestMsg.h>
#include <gobot_msg_srv/GetEncoders.h>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/GetIntArray.h>
#include <gobot_msg_srv/MotorSpeedMsg.h>
#include <gobot_msg_srv/EncodersMsg.h>


/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0);

/// get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

/// Initialize the serial connection
bool initSerial(void);

void initParams(ros::NodeHandle &nh);

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed);

bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Set the encoders to 0
bool resetEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool initialMotor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Just to test, we get the encoders and print them
bool testEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void mySigintHandler(int sig);