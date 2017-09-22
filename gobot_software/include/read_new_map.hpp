#ifndef READ_NEW_MAP
#define READ_NEW_MAP

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
#include <tf/transform_broadcaster.h>
#include "nav_msgs/OccupancyGrid.h"

#define PORT 5601

using boost::asio::ip::tcp;

/**
    Connects to the desktop application to receive maps
*/

/**
 * Main function that reads the new map with its id and metadata, and saves it
 */
void session(ros::NodeHandle n);

/**
 * Accepts the connection asynchronously
 */
void asyncAccept(ros::NodeHandle n);

/**
 * Called when we have been disconnected from the server
 */
void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif
