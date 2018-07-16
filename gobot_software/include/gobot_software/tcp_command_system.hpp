#ifndef TCP_COMMAND_SYSTEM
#define TCP_COMMAND_SYSTEM

#include <ros/ros.h>
#include <gobot_msg_srv/SendMessageToPc.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/MapMetaData.h>
#include <hector_exploration_node/Exploration.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>
#include <gobot_msg_srv/SendMap.h>

using boost::asio::ip::tcp;

template<typename Out>
void split(const std::string &s, const char delim, Out result);
bool execCommand(const std::string ip, const std::vector<std::string> command);

/*********************************** COMMAND FUNCTIONS ***********************************/
/// 4
bool textToSpeech(const std::vector<std::string> command);

/// 3
bool interruptDelay(const std::vector<std::string> command);

/// 1
bool adjustSpeed(const std::vector<std::string> command);

/// 2
bool adjustBatteryLvl(const std::vector<std::string> command);

/// a
bool renameRobot(const std::vector<std::string> command);

/// b
bool previousPath(const std::vector<std::string> command);

/// c
bool newGoal(const std::vector<std::string> command);

/// d
bool pausePath(const std::vector<std::string> command);

/// e
bool nextPath(const std::vector<std::string> command);

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

/// k
bool playPoint(const std::vector<std::string> command);

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
bool savePoints(const std::vector<std::string> command);

/// r

/// s
bool sendMapOnce(const std::string ip, const std::vector<std::string> command);

/// t
bool startNewScan(const std::string ip, const std::vector<std::string> command);

/// u
bool stopScanning(const std::string ip, const std::vector<std::string> command);

/// v
bool shutdownRobot(const std::vector<std::string> command);

/// w
bool muteOff(const std::vector<std::string> command);

/// x
bool muteOn(const std::vector<std::string> command);

/// y
bool setWifi(const std::vector<std::string> command);

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

void sendCommand(const std::string ip, const std::vector<std::string> command, std::string commandStr);

double RadToDegree(double rad);

double DegreeToRad(double degree);
/*********************************** SERVICES ***********************************/
bool pausePathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool playPathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopPathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool goDockSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopGoDockSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool highBatterySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool lowBatterySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool playPointSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool setPathSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool setSpeedSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);
    
bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
/*********************************** COMMUNICATION FUNCTIONS ***********************************/

void sendConnectionData(boost::shared_ptr<tcp::socket> sock);

bool sendMessageToSock(boost::shared_ptr<tcp::socket> sock, const std::string message);

bool sendMessageToAll(const std::string message);

void session(boost::shared_ptr<tcp::socket> sock);

void server(void);
/*********************************** DISCONNECTION FUNCTIONS ***********************************/
void mySigintHandler(int sig);

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

void disconnect(const std::string ip);


#endif
