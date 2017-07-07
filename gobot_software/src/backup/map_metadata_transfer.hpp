#ifndef MAP_METADATA_TRANSFER
#define MAP_METADATA_TRANSFER

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/String.h"
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
#include "std_srvs/Empty.h"

using boost::asio::ip::tcp;

/**
 * Send the metadata to the application
 */
void sendMetaData(const std::string& metadata_string);

/**
 * Called when metadata are published
 */
void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg);

/**
 * Service called to start listening to the metadata topic and transfer to the application
 */
bool startMeta(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

/**
 * Service called to stop listening to the metadata topic and transfer to the application
 */
bool stopMeta(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

#endif
