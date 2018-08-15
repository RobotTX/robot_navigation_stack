#ifndef TCP_TELEOP_CONTROL
#define TCP_TELEOP_CONTROL

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>
#include <gobot_msg_srv/GetStringArray.h>
#include <gobot_msg_srv/SetFloatArray.h>

using boost::asio::ip::tcp;

bool changeSpeedSrvCb(gobot_msg_srv::SetFloatArray::Request &req, gobot_msg_srv::SetFloatArray::Response &res);

void send_speed(int linear, int angular);

void tcp_teleop_control(const int8_t val);

void session(boost::shared_ptr<tcp::socket> sock);

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/
void timerCallback(const ros::TimerEvent&);

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif

