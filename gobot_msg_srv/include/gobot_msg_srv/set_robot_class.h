#ifndef SET_STATUS
#define SET_STATUS

//ros headers
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <gobot_msg_srv/robot_msgs.h>

#define PI_ 3.1415926

namespace robot_class {
    class SetRobot {
        public:
            SetRobot();
            ~SetRobot();
            void initialize();

            double degreeToRad(double degree);

            double radToDegree(double rad);

            double appToRobotYaw(double yaw, std::string unit = "deg");

            int robotToAppYaw(double yaw, std::string unit = "deg");

            int stopRobotMoving();

            bool setStatus(int status,std::string text);
            
            bool setDock(int status);

            bool setMode(const int mode);

            bool setStage(const int stage);

            bool setLoop(const int loop);

            bool setWifi(std::string wifi_name,std::string wifi_password);

            bool setVolume(const int volume);

            bool setName(std::string robot_name);

            bool setBatteryLvl(std::string battery_lvl);

            bool setSpeedLimit(std::string linear, std::string angular);

            bool setHome(std::string pos_x,std::string pos_y,std::string ori_x,std::string ori_y,std::string ori_z,std::string ori_w);

            bool clearPath(void);

            bool setNavSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);

            bool setMotorSpeed(char directionR, int velocityR, char directionL, int velocityL);

            void setInitialpose(const double p_x, const double p_y, const double q_x, const double q_y, const double q_z, const double q_w);

            void setInitialpose(const double p_x, const double p_y, const double p_yaw);
            
            void setInitialpose(geometry_msgs::PoseWithCovarianceStamped pose);

            void setBatteryLed();
            
            void setSound(int num,int time_on);

            void setLed(int mode, const std::vector<std::string> &color);
            
            void setMagnet(bool status);

            void setFootprint(std::string footprint);

            std::string killList();

            void reloadMap();
            
            void runNavi(bool simulation, bool reset_odom=true);

            void runScan(bool simulation, bool reset_odom=true);

            void speakEnglish(std::string str);

            void speakChinese(std::string str);

            void playSystemAudio(std::string str, int volume = -1);

            void killAudio();

            void changeVolume(int volume);

        private:
            std::string tts_en_, tts_ch_, voice_file_;
            std_srvs::Empty empty_srv;
            gobot_msg_srv::SetGobotStatus set_gobot_status_;
            gobot_msg_srv::SetInt set_dock_status_,set_stage_,set_loop_;
            gobot_msg_srv::MotorSpeedMsg motor_speed_;
            ros::Publisher speed_pub_, nav_pub_, sound_pub_, led_pub_, initial_pose_pub_;
    };
};

#endif
