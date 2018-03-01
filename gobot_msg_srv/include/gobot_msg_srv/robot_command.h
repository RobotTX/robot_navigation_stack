#ifndef ROBOT_COMMAND
#define ROBOT_COMMAND

#include <ros/ros.h>
#include <string>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>
#include <gobot_msg_srv/robot_msgs.h>

namespace robot_class {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    
    struct Pose{
        double x;
        double y;
        double theta;
        double qx,qy,qz,qw;
    };

    class RobotCommand {
        public:
            RobotCommand();
            ~RobotCommand();
            
            //Subscriber callback
            void encodersCallback(const gobot_msg_srv::EncodersMsg::ConstPtr& msg);
            
            //initialize class
            void initialize(tf::TransformListener* tf,costmap_2d::Costmap2DROS* global_costmap);

            //programmable voice
            void speakEnglish(std::string str);
            void speakChinese(std::string str);

            //Interact with base sensors such as color and sound
            void setLed(int mode, const std::vector<std::string> &color);
            void setSound(int num,int time_on);

            //Interact with base motors such as speed and encoder
            void setMotorSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
            void setMotorSpeed(const double linear_vel, const double angular_vel);
            void setSpeedLimit(std::string linear, std::string angular);
            void getEncoder(int &left_encoder, int &right_encoder);

            //RPY - Quaternion conversion
            void getYaw(double &yaw, const geometry_msgs::Quaternion &q);
            void getQuaternion(geometry_msgs::Quaternion &q, const double &yaw);

            //Localization
            void getCurrentPose(Pose &pose);

            void getCurrentPose(geometry_msgs::PoseStamped &pose);

            //Map
            void saveMapTo(std::string path = "");

            //Target Points || Routes

            //Obstacle Avoidance

        private:
            bool initialized_; 
            std_srvs::Empty empty_srv;
            costmap_2d::Costmap2DROS* global_costmap_;
            tf::TransformListener* tf_;
            MoveBaseClient* ac;
            Pose robot_pose_, goal_pose_;
            robot_class::SetRobot set_robot_;
            ros::Publisher vel_pub_;
            ros::Subscriber encoder_sub_;
            int left_encoder_, right_encoder_;
    };
};

#endif
