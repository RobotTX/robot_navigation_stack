#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <move_base_msgs/MoveBaseActionResult.h>

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

bool allowTebSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);