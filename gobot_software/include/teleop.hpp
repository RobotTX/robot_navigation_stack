#ifndef TELEOP
#define TELEOP

#include "ros/ros.h"
#include <iostream>
#include <cstdint>
#include <string>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "std_msgs/String.h"
#include <fstream>
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalID.h"


#define PORT 5602

using boost::asio::ip::tcp;

/**
 * Main function that reads the teleoperation key
 */
void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);

/**
 * Accepts the connection asynchronously
 */
void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n);

/**
 * Called when we have been disconnected from the server
 */
void serverDisconnected(const std_msgs::String::ConstPtr& msg);

/**
 * To teleoperate the robot, will publish on cmd_vel to make the robot move according to the given value
 */
void teleop(const int8_t val);

#endif

