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

using boost::asio::ip::tcp;

void send_speed(int linear, int angular);

void tcp_teleop_control(const int8_t val);

void session(boost::shared_ptr<tcp::socket> sock);

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif

