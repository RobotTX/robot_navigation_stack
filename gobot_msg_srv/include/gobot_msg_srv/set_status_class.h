#ifndef SET_STATUS
#define SET_STATUS

#include <ros/ros.h>
#include <string>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/SetDockStatus.h>
#include <gobot_msg_srv/GetDockStatus.h>
#include <gobot_msg_srv/SetStage.h>
#include <gobot_msg_srv/GetStage.h>
#include <gobot_msg_srv/SetPath.h>
#include <gobot_msg_srv/GetPath.h>
#include <gobot_msg_srv/SetString.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/GetInt.h>


namespace gobot_class {
    class SetStatus {
        public:
            SetStatus();
            bool setGobotStatus(int status,std::string text);
            
            bool setDockStatus(int status);

            bool setStage(const int stage);

            bool setLoop(const int loop);

            bool setWifi(std::string wifi_name,std::string wifi_password);

            bool setMute(const int mute);

            bool setName(std::string robot_name);

            bool setHome(std::string pos_x,std::string pos_y,std::string ori_x,std::string ori_y,std::string ori_z,std::string ori_w);

            bool clearPath(void);

            bool updatePath(void);

            void setSound(int num,int time_on, int time_off=1);

        private:
            std_srvs::Empty empty_srv;
            gobot_msg_srv::SetGobotStatus set_gobot_status;
            gobot_msg_srv::SetDockStatus set_dock_status;
            gobot_msg_srv::SetStage set_stage;

    };
};

#endif
