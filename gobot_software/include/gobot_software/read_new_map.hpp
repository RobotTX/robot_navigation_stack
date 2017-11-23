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
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gobot_msg_srv/SetString.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/SetStage.h>
#include <gobot_msg_srv/SetPath.h>
#include <gobot_msg_srv/GetGobotStatus.h>

using boost::asio::ip::tcp;

void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w,const double cov1,const double cov2);

void session(boost::shared_ptr<tcp::socket> sock);

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void);

/*********************************** DISCONNECTION FUNCTIONS ***********************************/
void mySigintHandler(int sig);

void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif
