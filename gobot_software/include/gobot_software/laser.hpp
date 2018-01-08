#ifndef LASER_TRANSFER
#define LASER_TRANSFER

#include <ros/ros.h>
#include <gobot_msg_srv/PortLaser.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;


void getLaserData(const sensor_msgs::LaserScan::ConstPtr& msg);

void sendLaserData(const std::vector<float>& scan);

bool sendLaserService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopSendLaserService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif