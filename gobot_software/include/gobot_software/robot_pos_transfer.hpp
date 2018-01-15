#ifndef ROBOT_POSE_TRANSFER
#define ROBOT_POSE_TRANSFER

#include <ros/ros.h>
#include <gobot_msg_srv/Port.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <tf/transform_broadcaster.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

/**
 * Send the robot position to the application
 */
void sendRobotPos(const ros::TimerEvent&);

/**
 * Called when robot position is published
 */
void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg);


/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/
void mySigintHandler(int sig);

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

void disconnect(const std::string ip);


#endif
