#ifndef ROBOT_POSE_TRANSFER
#define ROBOT_POSE_TRANSFER

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "geometry_msgs/PoseStamped.h"
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
#include <tf/transform_broadcaster.h>
#include "gobot_software/RecoveredPosition.h"

using boost::asio::ip::tcp;

/**
 * Send the robot position to the application
 */
void sendRobotPos(const std::string& robot_string);

/**
 * Called when robot position is published
 */
void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg);

/**
 * Service called to start the listening to the robot pos topic and transfer to the application
 */
bool startRobotPos(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

/**
 * Service called to stop the listening to the robot pos topic and transfer to the application
 */
bool stopRobotPos(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool confirmPositionRecovered(gobot_software::RecoveredPosition::Request &req, gobot_software::RecoveredPosition::Response &res);

void readAck(const std::string position_to_send);

#endif
