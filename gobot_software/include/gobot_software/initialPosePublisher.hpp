#ifndef INITIAL_POSE_PUBLISHER
#define INITIAL_POSE_PUBLISHER

#include <iostream>
#include <fstream>
#include <stdio.h>
#include "ros/ros.h"
#include <time.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gobot_msg_srv/IsCharging.h"


void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w);

#endif
