#ifndef MOVE_BASE_CONTROLLER
#define MOVE_BASE_CONTROLLER

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <hector_exploration_node/Exploration.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/SetString.h>

/// Call a service to get the trajectory for exploration from the hector_exploration_planner and send the goal to move_base
void doExploration(void);

/// Called when the service startExploration is called, launches doExploration in a new thread
bool startExplorationSrv(hector_exploration_node::Exploration::Request &req, hector_exploration_node::Exploration::Response &res);

/// Service to stop the exploration
bool stopExplorationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

/// To stop the exploration (either when the service is called or when we are done scanning)
bool stopExploration(void);

/// To make the robot go back to its starting position
void backToStart(void);

/// To get the starting position
bool getRobotPos(void);

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
    
void timerCallback(const ros::TimerEvent&);
#endif