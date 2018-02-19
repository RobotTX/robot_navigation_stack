#ifndef SET_STATUS
#define SET_STATUS

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/LedStrip.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <gobot_msg_srv/SetIntArray.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/SetStringArray.h>
#include <gobot_msg_srv/SetString.h>
#include <gobot_msg_srv/SetSpeeds.h>
#include <gobot_msg_srv/IsCharging.h>
#include <gobot_msg_srv/SetBattery.h>

namespace robot_class {
    class SetRobot {
        public:
            SetRobot();
            ~SetRobot();
            bool setStatus(int status,std::string text);
            
            bool setDock(int status);

            bool setStage(int stage);

            bool setLoop(int loop);

            bool setWifi(std::string wifi_name,std::string wifi_password);

            bool setMute(int mute);

            bool setName(std::string robot_name);

            bool setBatteryLvl(std::string battery_lvl);

            bool setSpeed(std::string linear, std::string angular);

            bool setHome(std::string pos_x,std::string pos_y,std::string ori_x,std::string ori_y,std::string ori_z,std::string ori_w);

            bool clearPath(void);

            bool setMotorSpeed(char directionR, int velocityR, char directionL, int velocityL);

            void setBatteryLed();
            
            void setSound(int num,int time_on, int mute=-1);

            void setLed(int mode, const std::vector<std::string> &color);
            
            std::string killList(bool simulation);

            void runNavi(bool simulation);

            void runScan(bool simulation);

            void speakEnglish(std::string str);

            void speakChinese(std::string str);

        private:
            std::string tts_en_, tts_ch_;
            std_srvs::Empty empty_srv;
            gobot_msg_srv::SetGobotStatus set_gobot_status_;
            gobot_msg_srv::SetInt set_dock_status_,set_stage_,set_loop_;
            gobot_msg_srv::SetSpeeds motor_speed_;
    };
};

#endif
