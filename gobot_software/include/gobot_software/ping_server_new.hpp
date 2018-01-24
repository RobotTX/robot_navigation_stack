#ifndef PING_SERVER_HPP
#define PING_SERVER_HPP

#include <iostream>
#include <string>
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>
#include <fstream>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/GetDockStatus.h>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetStage.h>
#include <gobot_msg_srv/GetStage.h>
#include <gobot_msg_srv/SetString.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_software/timeout_blocking_tcp_client.h>
#include <mutex>
#include <thread>

using boost::asio::ip::tcp;

std::string getDataToSend(void);

bool disServersSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);

bool updataStatusSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void pingAllIPs(void);

void pingIP(std::string ip, std::string dataToSend, double sec);

void pingIP2(std::string ip, std::string dataToSend, double sec);

void checkNewServers(void);

void updateIP(void);

bool initParams(void);

void mySigintHandler(int sig);
#endif
