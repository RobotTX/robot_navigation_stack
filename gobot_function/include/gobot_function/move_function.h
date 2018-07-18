#ifndef MOVE_FUNCTION
#define MOVE_FUNCTION

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <gobot_msg_srv/get_robot_class.h>
#include <gobot_msg_srv/robot_move_class.h>

struct PathPoint {
    robot_class::Point p;
    bool isHome;
    double waitingTime;
    std::string text;
    double delayText;
    int audioIndex;
};


//****************************** CALLBACK ******************************
/*
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
*/
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg);

//****************************** SERVICE ******************************
bool playPointService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool updatePathService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res);

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool goDockAfterPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool interruptDelayService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool skipPathService(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res);

//****************************** FUNCTIONS ******************************
void goalReached();

void moveToGoal(PathPoint goal);

void checkSound();

void checkGoalDelay();

void goDockAfterPath();

void textToSpeech(std::string text, double delay);

void playAudio(std::string file, double delay);

void readPath(std::vector<std::string> &path);

void initData();

void mySigintHandler(int sig);
#endif