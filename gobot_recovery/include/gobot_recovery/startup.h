#include <ros/ros.h>
#include <signal.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

void mySigintHandler(int sig);