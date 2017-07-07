#ifndef COMMAND_SYSTEM
#define COMMAND_SYSTEM

#include <ros/ros.h>
#include <gobot_software/Port.h>
#include <gobot_software/PortLaser.h>
#include <gobot_software/SendMessageToPc.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
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
#include <boost/regex.hpp>
#include <list>
#include <fstream>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <nav_msgs/MapMetaData.h>
#include <gobot_software/SetDockStatus.h>
#include <gobot_software/GetDockStatus.h>


#define CMD_PORT 5600

using boost::asio::ip::tcp;

template<typename Out>
void split(const std::string &s, const char delim, Out result);
bool execCommand(ros::NodeHandle n, const std::vector<std::string> command);
void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);
void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n);
void startRobotPos(void);
void stopRobotPos(void);
bool startMap(void);
bool sendOnceMap(const int who);
bool sendAutoMap(void);
bool stopAutoMap(void);
bool stopMap(void);
void server(const unsigned short port, ros::NodeHandle n);
void getPorts(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);
bool sendMessageToPc(boost::shared_ptr<tcp::socket> sock, const std::string message);
bool startLaserData(const bool startLaser);
bool sendLaserData(void);
bool stopSendLaserData(void);
bool stopLaserData(void);
bool stopParticleCloudData(void);
void stopTwist(void);
bool recoverPosition(void);
bool stopRecoveringPosition(void);
bool sendLocalMap(void);
void disconnect(void);
bool resumeRecoveryPosition(void);
bool setDockStatus(gobot_software::SetDockStatus::Request &req, gobot_software::SetDockStatus::Response &res);
bool getDockStatus(gobot_software::GetDockStatus::Request &req, gobot_software::GetDockStatus::Response &res);

#endif
