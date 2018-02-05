#ifndef GET_STATUS
#define GET_STATUS

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/GetIntArray.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/GetStringArray.h>
#include <gobot_msg_srv/GetString.h>


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

            std::string getName();

        private:
            gobot_msg_srv::GetGobotStatus get_gobot_status_;
            gobot_msg_srv::GetInt get_dock_status_;

    };
};

#endif
