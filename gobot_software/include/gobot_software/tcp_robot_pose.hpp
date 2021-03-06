#ifndef TCP_ROBOT_POSE
#define TCP_ROBOT_POSE

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>
#include <gobot_msg_srv/GetStringArray.h>

using boost::asio::ip::tcp;

/**
 * Send the robot position to the application
 */
void sendRobotPos(const ros::TimerEvent&);

/**
 * Called when robot position is published
 */
void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg);

bool getRobotPoseSrvCb(gobot_msg_srv::GetStringArray::Request &req, gobot_msg_srv::GetStringArray::Response &res);

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/
void mySigintHandler(int sig);

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

void disconnect(const std::string ip);

void initParams();

#endif
