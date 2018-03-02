/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TENG Xiao
 *********************************************************************/
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
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

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

    struct Path{
        std::string name;
        double x;
        double y;
        int waiting;
        double theta;
    };

    class RobotCommand {
        public:
            RobotCommand();
            ~RobotCommand();
            
            /**
            * @brief  Subscriber callback
            */
            void encodersCallback(const gobot_msg_srv::EncodersMsg::ConstPtr& msg);
            void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed);
            void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg);
            void laserCallback(const sensor_msgs::LaserScan msg);
            void globalPathCallback(const nav_msgs::Path msg);
            void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);
            void sonarCallback(const gobot_msg_srv::SonarMsg msg);

            //@Initialize class
            //void initialize(tf::TransformListener* tf,costmap_2d::Costmap2DROS* global_costmap);
            void initialize();
            

            /**
            * @brief  Programmable voice
            * @param str The text for robot to convert to speech
            */
            void speakEnglish(std::string str);
            void speakChinese(std::string str);

            //@Interact with base sensors such as color and sound
            void setLed(int mode, const std::vector<std::string> &color);
            void setSound(int num,int time_on);
            int getBatteryPercent();
            int getBatteryChargingCurrent();
            bool getBatteryCharging();


            //@Interact with base motors such as speed and encoder
            void setMotorSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL);
            void setMotorSpeed(const double linear_vel, const double angular_vel);
            void setSpeedLimit(std::string linear, std::string angular);
            void getEncoder(int &left_encoder, int &right_encoder);
            void getMotorSpeed(int &left_speed, int &right_speed);

            //@RPY - Quaternion conversion
            void getYaw(double &yaw, const geometry_msgs::Quaternion &q);
            void getQuaternion(geometry_msgs::Quaternion &q, const double &yaw);

            //@Localization
            void getCurrentPose(Pose &pose);
            void getCurrentPose(geometry_msgs::PoseStamped &pose);

            //Map
            void saveMapTo(std::string path = "");

            //@Target Points || Routes
            //get the point cost in costmap
            //costmap_2d::FREE_SPACE=0, costmap_2d::INSCRIBED_INFLATED_OBSTACLE=253, costmap_2d::LETHAL_OBSTACLE=254, costmap_2d::NO_INFORMATION=255
            int getPointCost(const int x,const int y);
            //First param = k, 2nd is point name, 3rd is x coordinate, 4th is y coordinate, 5th is orientation, 6th is home bool
            bool setTargetPoint(const Pose &point);
            bool setTargetPoint(const geometry_msgs::PoseStamped &point);
            /// First param = i, then the path name, then quadriplets of parameters to represent path points (path, point name, posX, posY, waiting time,orientation) 
            bool setTargetPath(const std::vector<Path> &path, std::string path_name="default");
            void getCurrentGoal(geometry_msgs::PoseStamped &goal);

            //@Control robot motion (play, pause, stop)
            void playTargetPath();
            void pauseRobot();
            void stopRobot();
            void startDock();
            void stopDock();
            void shutDown();

            //@Obstacle Detection & Avoidance
            void getLaserData(sensor_msgs::LaserScan &data);
            void getSonarData(std::vector<int> &data);
            void getPlanPath(std::vector<geometry_msgs::PoseStamped> &plan_path);
            void makePlanPath(const geometry_msgs::PoseStamped &start,const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path);
            void makePlanPath(const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path);
            void clearCostMap();

        private:
            bool initialized_; 
            std_srvs::Empty empty_srv_;
            costmap_2d::Costmap2DROS* global_costmap_;
            tf::TransformListener* tf_;
            Pose robot_pose_, goal_pose_;
            robot_class::SetRobot set_robot_;
            ros::Publisher vel_pub_;
            ros::Subscriber encoder_sub_, speed_sub_, battery_sub_, laser_sub_, global_path_sub_, goal_sub_, sonar_sub_;
            sensor_msgs::LaserScan laser_data_;
            geometry_msgs::PoseStamped current_goal_;
            nav_msgs::Path global_path_;
            gobot_msg_srv::SonarMsg sonar_data_;
            int left_encoder_, right_encoder_;
            int left_speed_, right_speed_;
            int battery_percent_, charging_current_; 
            bool charging_flag_;
            
    };
};

#endif
