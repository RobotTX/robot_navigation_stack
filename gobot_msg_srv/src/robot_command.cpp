#include <gobot_msg_srv/robot_command.h>


namespace robot_class {

    RobotCommand::RobotCommand():SetRobot(){

    }
    RobotCommand::~RobotCommand(){};

    void RobotCommand::initilize(){
        ros::NodeHandle n;

        pose_sub_ = n.subscribe("/robot_pose", 1, &RobotCommand::PoseCallback, this);
    }

    void RobotCommand::getPose(Pose &current_pose){
        current_pose = robot_pose_;
    }


    bool sendGoal(const Pose &goal){
        /*
        gobot_msg_srv::SetStringArray set_point;    
        set_point.request.data.push_back("robot_command");
        set_point.request.data.push_back();
        set_point.request.data.push_back();
        set_point.request.data.push_back();
        set_point.request.data.push_back("0");
        return ros::service::call("/gobot_function/play_point", set_point);
        */
    }
    bool sendGoal(const double &dis, const double &theta){
        //quaternion.setRPY(0, 0, -(currentGoal.yaw+90)*3.14159/180);
    }
    
    //callback function
    void RobotCommand::PoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
        robot_pose_.x = msg->position.x;
        robot_pose_.y = msg->position.y;
        robot_pose_.qx = msg->orientation.x;
        robot_pose_.qy = msg->orientation.y;
        robot_pose_.qz = msg->orientation.z;
        robot_pose_.qw = msg->orientation.w;
        robot_pose_.theta = tf::getYaw(tf::Quaternion(robot_pose_.qx , robot_pose_.qy , robot_pose_.qz, robot_pose_.qw));
    }
};

