#ifndef READ_NEW_MAP
#define READ_NEW_MAP

#include <ros/ros.h>
#include <iostream>
#include <cstdint>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

void session(boost::shared_ptr<tcp::socket> sock);

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif
