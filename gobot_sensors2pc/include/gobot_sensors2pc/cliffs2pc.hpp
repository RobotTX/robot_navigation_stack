#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "pcl_ros/point_cloud.h"
#include <gobot_msg_srv/set_robot_class.h>

void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff);
bool cliffToCloud(double CliffData,pcl::PointCloud<pcl::PointXYZ> &cloudData);
void initParams();