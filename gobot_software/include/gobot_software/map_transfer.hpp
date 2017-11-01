#ifndef MAP_TRANSFER
#define MAP_TRANSFER

#include <ros/ros.h>
#include <gobot_msg_srv/SendMap.h>
#include <gobot_msg_srv/SetString.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;


/**
 * Send the map to the application as pixels (0 to 255)
 */
void sendMap(const std::string ip, const std::vector<uint8_t>& my_map);

/**
 * Called when metadata are published
 */
void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg);

/**
 * Called when map is published
 */
void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

/**
 * Compresses the map in order to send it faster
 */
std::vector<uint8_t> compress(const std::vector<int8_t> map, const int map_width, const int map_height, const double map_resolution, const double _map_orixin_x, const double _map_orixin_y, const int who);

/**
 * Service called to start sending the map to the app (constantly)
 */
bool sendAutoMap(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);

/**
 * Service called to stop sending the map to the app
 */
bool stopAutoMap(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res);

/* who:
 * 0 : scan
 * 1 : application requesting at connection time
 * 2 : to merge
 * Service called to send the map once to the app 
 * (for example when the map is requested from the robot to be used inside the application)
 */
bool sendOnceMap(gobot_msg_srv::SendMap::Request &req,
    gobot_msg_srv::SendMap::Response &res);

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/
void mySigintHandler(int sig);

void serverDisconnected(const std_msgs::String::ConstPtr& msg);


#endif
