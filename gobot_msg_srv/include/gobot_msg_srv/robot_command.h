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
 *   * The names of its contributors may be used to endorse or promote 
 *     products derived from this software without specific prior written 
 *     permission.
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
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/MapMetaData.h>

#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>
#include <gobot_msg_srv/robot_msgs.h>

namespace robot_class {
    
    struct Pose{
        double x;
        double y;
        //unit rads
        double theta;
    };

    struct Path{
        std::string name;
        double x;
        double y;
        int waiting;
        double theta;
    };

    struct Map{
        float resolution;
        int pixel_width;
        int pixel_height;
        float meter_width;
        float meter_height;
        Pose origin;
    };

    struct Gyro{
        int gyrox;
        int gyroy;
        int gyroz;
        int accelx;
        int accely;
        int accelz;
    };

    class RobotCommand {
        public:
            /**
            * @BRIEFT  Class constructor
            */
            RobotCommand();

            /**
            * @BRIEFT  Class destructor
            */
            ~RobotCommand();


            /**
            * @BRIEFT  Initialize class object
            */
            //void initialize(tf::TransformListener* tf,costmap_2d::Costmap2DROS* global_costmap);
            void initialize();

            /**
            * @brief stops costmap updates and unsubscribes from sensor topics. 
            */
            void stopCostmap();

            /**
            * @brief subscribes to sensor topics if necessary and starts costmap updates
            */
            void startCostmap();


            /**
            * @BRIEFT  Subscribers callback
            */

            /**
            * @brief encoders callback function
            */
            void encodersCallback(const gobot_msg_srv::EncodersMsg::ConstPtr& msg);

            /**
            * @brief motor speeds callback function
            */
            void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed);

            /**
            * @brief battery callback function
            */
            void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg);

            /**
            * @brief laser callback function
            */
            void laserCallback(const sensor_msgs::LaserScan msg);

            /**
            * @brief planned path callback function
            */
            void globalPathCallback(const nav_msgs::Path msg);

            /**
            * @brief goal callback function
            */
            void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

            /**
            * @brief sonars callback function
            */
            void sonarCallback(const gobot_msg_srv::SonarMsg msg);

            /**
            * @brief gyro and accelerometer callback function
            */
            void gyroCallback(const gobot_msg_srv::GyroMsg::ConstPtr& msg);

            /**
            * @brief goal status callback function
            */
            void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
            
            /**
            * @brief map metadata status callback function
            */
            void mapDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);


            /**
            * @BRIEFT  Programmable voice
            * @param str: text for robot to convert to speech
            */
            void ttsEnglish(std::string str);
            void ttsChinese(std::string str);


            /**
            * @BRIEFT  Interact with base sensors such as color, sound and sensors data
            */

            /**
            * @param mode: 0 means permanent LED display; 1 means running LED display
            * @param color: LED colors to display (mode 0 needs only 1 color input, and mode 2 needs at least 2 colors input)
            * *color value: "green", "blue", "yellow", "red", "cyan", "white", "magenta", "off"
            */
            void setLed(int mode, const std::vector<std::string> &color);

            /**
            * @param num: how many times of beeps for the buzzer 
            * @param time_on: how long each beep last for
            */
            void setSound(int num,int time_on);

            /**
            * @brief return current battery percentage ranging from 0 to 100 (integer)
            */
            int getBatteryPercent();

            /**
            * @brief return current battery charging current (integer); 
            * *usually positive values when charging, and negative values when not charging;
            */
            int getBatteryChargingCurrent();

            /**
            * @brief return current charging status; true indicates charging
            */
            bool getBatteryCharging();

            /**
            * @param gyro: get the current gyroscope and accelerometer data (self-defined type 'GyroMsg')
            */
            void getGyro(Gyro &gyro_data);


            /**
            * @BRIEFT  Interact with base motors such as get/set motor speeds and encoders
            */

            /**
            * @brief two ways to set robot moving speed:
            * *1.set direction and velocity for the robot's each wheel
            * *2.set linear and angular velocities for the whole robot
            */
            /**
            * @param directionR: set turning direction for right wheel, where 'F' indicates forward; 'B' indicates backwards;
            * @param velocityR: set turning velocity for right wheel, where value ranges from 0 to 127 
            * @param directionL, velocityL: same as before for setting left wheel's turning direction and velocity
            */
            void setMotorSpeed(const char directionL, const int velocityL, const char directionR, const int velocityR);
            /**
            * @param linear_vel: set linear velocity of robot
            * @param angular_vel: set angular velocity of robot
            */
            void setMotorSpeed(const double linear_vel, const double angular_vel);

            /**
            * @param linear: set limit for linear velocity with unit m/s (recommanded value: 0.4m/s)
            * @param angular: set limit for angular velocity with unit rad/s (recommanded value: 0.8 rad/s)
            */
            void setSpeedLimit(std::string linear, std::string angular);

            /**
            * @param left_encoder: get left encoder's accumulated reading
            * @param right_encoder: get right encoder's accumulated reading
            */
            void getEncoder(int &left_encoder, int &right_encoder);

            /**
            * @param left_speed: get left wheel's speed ranging from -127 to 127 (128 indicates out of range)
            * @param right_speed: get right wheel's speed ranging from -127 to 127 (128 indicates out of range)
            */
            void getMotorSpeed(int &left_speed, int &right_speed);


            /**
            * @BRIEFT  Quaternion and yaw conversion
            */

            /**
            * @param yaw: get the yaw by converting given quaternion
            * @param q: set the quaternion for yaw convertion
            */
            void getYaw(double &yaw, const geometry_msgs::Quaternion &q);

            /**
            * @param q: get the quaternion by converting given yaw
            * @param yaw: set the yaw for quaternion convertion
            */
            void getQuaternion(geometry_msgs::Quaternion &q, const double &yaw);


            /**
            * @BRIEFT  Localization
            */

            /**
            * @brief two ways to get robot current pose in the global frame:
            * *1.self-defined data type 'Pose'
            * *2.ROS-defined data type 'geometry_msgs::PoseStamped'
            */
            /**
            * @param pose: get the current pose as type 'Pose' 
            */
            void getCurrentPose(Pose &pose);

            /**
            * @param pose: get the current pose as type 'geometry_msgs::PoseStamped'
            */
            void getCurrentPose(geometry_msgs::PoseStamped &pose);

            /**
            * @BRIEFT  Map
            */

            /**
            * @param path: set the path to store robot's map to; if no path given, save map to the same path as the node's
            */
            void saveMapTo(std::string path = "");

            /**
            * @param data: get current map's data such as resolution, width, height and origin
            * *if map is not initialized, return false
            */
            void getMapMetadata(Map &data);


            /**
            * @BRIEFT  Target Points || Routes
            */

            /**
            * @brief get the grid cost for the given position (x,y)
            * @param x: x coordinate for given position
            * @param y: y coordinate for given position
            * @comment: cost value ranges from 0 to 255, meaning: 
            * *costmap_2d::FREE_SPACE=0, 
            * *costmap_2d::INSCRIBED_INFLATED_OBSTACLE=253, 
            * *costmap_2d::LETHAL_OBSTACLE=254, 
            * *costmap_2d::NO_INFORMATION=255
            */
            int getPointCost(const double x,const double y);

            /**
            * @brief two ways to send a goal point for robot:
            * *1.self-defined data type 'Pose'
            * *2.ROS-defined data type 'geometry_msgs::PoseStamped'
            */
            /**
            * @param point: set the goal point as type 'Pose' 
            */
            bool setTargetPoint(const Pose &point);

            /**
            * @param point: set the goal point as type 'geometry_msgs::PoseStamped' 
            */
            bool setTargetPoint(const geometry_msgs::PoseStamped &point);

            /**
            * @brief set a route for robot
            * @param path: set the route as self-defined type 'Path' 
            * @param path_name: set the name for the route; if no name given, set route's name to be "default"
            */
            bool setTargetPath(const std::vector<Path> &path, std::string path_name="default");

            /**
            * @brief get current goal of robot
            * @param goal: get the current goal information if have as type 'geometry_msgs::PoseStamped'
            */
            void getCurrentGoal(geometry_msgs::PoseStamped &goal);

            /**
            * @brief return goal status
            * *-1 - No goal
            * * 1 - Goal active
            * * 3 - Goal complete
            * * 4 - Goal aborted
            */
            int getGoalStatus();


            /**
            * @BRIEFT  Control robot motion
            */

            /**
            * @brief start robot to execute assigned route if have
            */
            void playTargetPath();

            /**
            * @brief stop robot; when re-executing route, robot will go to the uncompleted point
            */
            void pauseRobot();

            /**
            * @brief stop robot; when re-executing route, robot will go to first point 
            */
            void stopRobot();

            /**
            * @brief start robot to go to docking station
            */
            void startDock();

            /**
            * @brief stop robot from going to docking station
            */
            void stopDock();

            /**
            * @brief poweroff robot and electrical system
            */
            void shutDown();


            /**
            * @BRIEFT  Obstacle Detection & Avoidance
            */

            /**
            * @brief get laser data
            * @param data: get the laser data as type 'sensor_msgs::LaserScan'
            */
            void getLaserData(sensor_msgs::LaserScan &data);

            /**
            * @brief get four sonars data (two in front and two in rear)
            * @param data: get the sonars data as type 'std::vector<int>', where four sonars data are stored in data[0], data[1], data[2] and data[3] respectively
            */
            void getSonarData(std::vector<int> &data);

            /**
            * @brief get planned path to goal pose if have
            * @param plan_path: get the planned path consisting of 'geometry_msgs::PoseStamped' points
            */
            void getPlanPath(std::vector<geometry_msgs::PoseStamped> &plan_path);

            /**
            * @brief plan path for given start pose and goal pose
            * *return true if have plan 
            * *return false if no plan
            * @param plan_path: get the planned path consisting of 'geometry_msgs::PoseStamped' points
            */
            bool makePlanPath(const geometry_msgs::PoseStamped &start,const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path);

            /**
            * @brief get planned path from current pose to given goal pose if have
            * *return true if have plan 
            * *return false if no plan
            * @param plan_path: get the planned path consisting of 'geometry_msgs::PoseStamped' points
            */
            bool makePlanPath(const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path);

            /**
            * @brief clear obstacles information in costmap. 
            * *As a result, only static map information will be kept after calling this function
            */
            void clearCostMap();

        private:
            bool initialized_; 
            bool charging_flag_;
            bool map_received_;
            int left_encoder_, right_encoder_;
            int left_speed_, right_speed_;
            int battery_percent_, charging_current_; 
            int goal_status_;

            ros::Publisher vel_pub_;
            ros::Subscriber encoder_sub_, speed_sub_, battery_sub_, laser_sub_, global_path_sub_, goal_sub_, sonar_sub_, gyro_sub_, goal_status_sub_, map_sub_;

            std_srvs::Empty empty_srv_;
            costmap_2d::Costmap2DROS* global_costmap_;
            tf::TransformListener* tf_;
            nav_msgs::Path global_path_;
            sensor_msgs::LaserScan laser_data_;
            geometry_msgs::PoseStamped current_goal_;
            
            robot_class::SetRobot set_robot_;
            Pose robot_pose_, goal_pose_;
            Map map_data_;
            Gyro gyro_data_;
            gobot_msg_srv::SonarMsg sonar_data_;
    };
};

#endif
