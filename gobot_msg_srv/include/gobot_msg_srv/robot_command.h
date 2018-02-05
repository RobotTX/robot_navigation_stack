#ifndef ROBOT_COMMAND
#define ROBOT_COMMAND

#include <ros/ros.h>
#include <string>
#include <std_srvs/Empty.h>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>


#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>

namespace robot_class {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    
    struct Pose{
        double x;
        double y;
        double theta;
        double qx,qy,qz,qw;
    };

    class RobotCommand: public SetRobot{
        public:
            RobotCommand();
            ~RobotCommand();
            
            void initilize();

            //math related command
            

            //motion related command
            void getPose(Pose &current_pose);
            void getGoal(Pose &goal_pose);
            bool sendGoal(const double &x, const double &y, const double &theta);
            bool sendGoal(const Pose &goal);
            bool sendGoal(const double &dis, const double &theta);

            //sensor related command
            void getSonar(double &s1, double &s2, double &s3, double &s4);
            void getCliff(double &c1, double &c2, double &c3, double &c4);
            void getOdom(int &left_wheel, int &right_wheel);
            bool isCharging();
            int batteryPercent();

            //speed related command
            void getRealSpeed(double &linear,double &angular);
            void getCommandSpeed(double &linear,double &angular);
            void sendMotionSpeed(const double &linear, const double &angular);
            void sendWheelSpeed(const double &left_wheel, const double &right_wheel);

            //communication related command
            std::string getIP();

            //callback functions
            void PoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

        private:
            std_srvs::Empty empty_srv;
            Pose robot_pose_, goal_pose_;
            ros::Subscriber pose_sub_;
    };
};

#endif
