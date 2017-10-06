#ifndef COMMAND_SYSTEM
#define COMMAND_SYSTEM

#include <ros/ros.h>
#include <gobot_msg_srv/Port.h>
#include <gobot_msg_srv/PortLaser.h>
#include <gobot_msg_srv/SendMessageToPc.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <nav_msgs/MapMetaData.h>
#include <gobot_msg_srv/SetDockStatus.h>
#include <gobot_msg_srv/GetDockStatus.h>
#include <gobot_msg_srv/String.h>
#include <gobot_msg_srv/SendMap.h>
#include <hector_exploration_node/Exploration.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>


#define CMD_PORT 5600

using boost::asio::ip::tcp;

template<typename Out>
void split(const std::string &s, const char delim, Out result);
bool execCommand(const std::string ip, const std::vector<std::string> command);

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
bool playScan(const std::string ip, const std::vector<std::string> command);

/// f
bool pauseScan(const std::string ip, const std::vector<std::string> command);

/// g
bool startScanAndAutoExplore(const std::string ip, const std::vector<std::string> command);

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
bool sendMapOnce(const std::string ip, const std::vector<std::string> command);

/// t
bool startNewScan(const std::string ip, const std::vector<std::string> command);

/// u
bool stopScanning(const std::string ip, const std::vector<std::string> command);

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

bool sendMapAutomatically(const std::string ip);

bool stopSendingMapAutomatically(const std::string ip);

bool goDock(void);


/*********************************** SERVICES ***********************************/

bool setDockStatus(gobot_msg_srv::SetDockStatus::Request &req, gobot_msg_srv::SetDockStatus::Response &res);

bool getDockStatus(gobot_msg_srv::GetDockStatus::Request &req, gobot_msg_srv::GetDockStatus::Response &res);

bool goDockService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool lowBattery(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);


/*********************************** COMMUNICATION FUNCTIONS ***********************************/

void sendConnectionData(boost::shared_ptr<tcp::socket> sock);

bool sendMessageToSock(boost::shared_ptr<tcp::socket> sock, const std::string message);

bool sendMessageToAll(const std::string message);

void session(boost::shared_ptr<tcp::socket> sock);

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

void disconnect(const std::string ip);


#endif
