#include <ros/ros.h>
#include <signal.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

void initialPoseCallback(const std_msgs::Int8::ConstPtr& msg);

void mySigintHandler(int sig);