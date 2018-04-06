//ros headers
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <numeric> 
#include <serial/serial.h>
#include <thread>
#include <serial/serial.h>

#include <gobot_msg_srv/robot_msgs.h>


/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0);

/// get the output of the given system command
std::string getStdoutFromCommand(std::string cmd);

/// Initialize the serial connection
bool initSerial(void);

void initParams(ros::NodeHandle &nh);

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed);

bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// Just to test, we get the encoders and print them
bool testEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void mySigintHandler(int sig);