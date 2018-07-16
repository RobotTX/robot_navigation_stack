#ifndef ROBOT_MOVE
#define ROBOT_MOVE

//ros headers
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <gobot_msg_srv/set_robot_class.h>



class RobotMove: public robot_class::SetRobot {
    public:
        RobotMove();
        ~RobotMove();
    private:
};

#endif
