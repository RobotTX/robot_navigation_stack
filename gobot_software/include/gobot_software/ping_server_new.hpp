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
#include <gobot_msg_srv/GetDockStatus.h>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_software/timeout_blocking_tcp_client.h>
#include <mutex>
#include <thread>

using boost::asio::ip::tcp;

std::string getDataToSend(void);

void pingAllIPs(void);

void pingIP(std::string ip, std::string dataToSend);

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo);

void checkNewServers(void);

void updateIP(void);

#endif
