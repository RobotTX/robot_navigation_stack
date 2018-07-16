#ifndef GET_STATUS
#define GET_STATUS

//ros headers
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>

//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <numeric> 

#include <gobot_msg_srv/robot_msgs.h>

namespace robot_class {
    class GetRobot {
        public:
            GetRobot();
            ~GetRobot();
            
            void getStatus(int &status,std::string &text);
            void getStatus(int &status);
            void getHome(double &x, double &y, double &qx, double &qy, double &qz, double &qw);

            int getDock();
            int getLoop();
            int getStage();
            int getMute();

            std::string getName();

        private:
            gobot_msg_srv::GetGobotStatus get_gobot_status_;
            gobot_msg_srv::GetInt get_dock_status_;

    };
};

#endif
