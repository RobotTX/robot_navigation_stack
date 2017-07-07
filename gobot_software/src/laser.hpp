#ifndef LASER_TRANSFER
#define LASER_TRANSFER

#include "ros/ros.h"
#include "gobot_software/PortLaser.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdint>
#include <vector>
#include <cstdlib>
#include <signal.h>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <thread>
#include "std_srvs/Empty.h"
#include "std_msgs/Float32.h"

using boost::asio::ip::tcp;

bool startLaser(gobot_software::PortLaser::Request &req, gobot_software::PortLaser::Response &res);

void getLaserData(const sensor_msgs::LaserScan::ConstPtr& msg);

void sendLaserData(const std::vector<float>& scan);

bool stopSendingLaserData(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool sendLaser(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopSendLaser(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

#endif