#ifndef MAP_TRANSFER
#define MAP_TRANSFER

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
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
#include <chrono>
#include <thread>
#include "std_srvs/Empty.h"

using boost::asio::ip::tcp;

/**
 * Send the map to the application as pixels (0 to 255)
 */
void sendMap(const std::vector<uint8_t>& my_map);

/**
 * Send the real map to the software as percentage of chance of a wall (-1 to 100)
 */
void sendMap(const std::vector<int8_t>& my_map);

/**
 * Called when map is published
 */
void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

/**
 * Service called to start the socket connection
 */
bool startMap(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

/**
 * Service called to stop everything (in particular it closes the connection)
 */
bool stopMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/**
 * Service called to start sending the map to the app (constantly)
 */
bool sendAutoMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/**
 * Service called to stop sending the map to the app
 */
bool stopAutoMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/**
 * Service called to send the map once to the app 
 * (for example when the map is requested from the robot to be used inside the application, or to merge maps)
 */
bool sendOnceMap(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

/**
 * Compresses the map in order to send it faster
 */
std::vector<uint8_t> compress(const std::vector<int8_t> map, const int map_width, const int map_height, const int who);

/**
 * Send local map when the robot is recovering its position
 */
bool sendLocalMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/**
 * Called when local map is published
 */
void getLocalMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

/**
 * Called when metadata are published
 */
void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg);

#endif
