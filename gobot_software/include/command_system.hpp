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
#include <hector_exploration_node/Exploration.h>


#define CMD_PORT 5600

using boost::asio::ip::tcp;

template<typename Out>
void split(const std::string &s, const char delim, Out result);
bool execCommand(const std::vector<std::string> command);

/*********************************** COMMAND FUNCTIONS ***********************************/

/// a
bool renameRobot(const std::vector<std::string> command);

/// NOT USED ANYMORE
/// b
bool changeWifi(const std::vector<std::string> command);

/// c
bool newGoal(const std::vector<std::string> command);

/// d
bool pausePath(const std::vector<std::string> command);

/// e
bool playScan(const std::vector<std::string> command);

/// f
bool pauseScan(const std::vector<std::string> command);

/// g
bool startScanAndAutoExplore(const std::vector<std::string> command);

/// h
bool robotStartup(const std::vector<std::string> command);

/// i
bool newPath(const std::vector<std::string> command);

/// j
bool playPath(const std::vector<std::string> command);

/// NOT USED ANYMORE
/// k
bool deletePath(const std::vector<std::string> command);

/// l
bool stopPath(const std::vector<std::string> command);

/// m
bool stopAndDeletePath(const std::vector<std::string> command);

/// n
bool newChargingStation(const std::vector<std::string> command);

/// o
bool goToChargingStation(const std::vector<std::string> command);

/// p
bool stopGoingToChargingStation(const std::vector<std::string> command);

/// q
bool sendLaserData(const std::vector<std::string> command);

/// r
bool stopSendingLaserData(const std::vector<std::string> command);

/// s
bool sendMapOnce(const std::vector<std::string> command);

/// t
bool startNewScan(const std::vector<std::string> command);

/// u
bool stopScanning(const std::vector<std::string> command);

/// v
bool recoverPosition(const std::vector<std::string> command);

/// w
bool pauseRecoveringPosition(const std::vector<std::string> command);

/// x
bool stopRecoveringPosition(const std::vector<std::string> command);

/// y
bool resumeRecoveringPosition(const std::vector<std::string> command);

/// z
bool restartEverything(const std::vector<std::string> command);

/// ,
bool startAutoExplore(const std::vector<std::string> command);

/// .
bool stopAutoExplore(const std::vector<std::string> command);

/// /
bool loopPath(const std::vector<std::string> command);


/*********************************** SOME FUNCTIONS USED MULTIPLE TIMES ***********************************/

bool sendMapAutomatically(void);

bool stopSendingMapAutomatically(void);


/*********************************** SERVICES ***********************************/

bool setDockStatus(gobot_software::SetDockStatus::Request &req, gobot_software::SetDockStatus::Response &res);

bool getDockStatus(gobot_software::GetDockStatus::Request &req, gobot_software::GetDockStatus::Response &res);


/*********************************** STARTUP CONNECTION FUNCTIONS ***********************************/

void startRobotPosConnection(void);

void stopRobotPosConnection(void);

bool startMapConnection(void);

bool stopMapConnection(void);

bool startLaserDataConnection(const bool startLaser);

bool stopLaserDataConnection(void);

/*********************************** COMMUNICATION FUNCTIONS ***********************************/

void getPorts(boost::shared_ptr<tcp::socket> sock);

void session(boost::shared_ptr<tcp::socket> sock);

bool sendMessageToPc(boost::shared_ptr<tcp::socket> sock, const std::string message);

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor);

void server(const unsigned short port);

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

void disconnect(void);


#endif
