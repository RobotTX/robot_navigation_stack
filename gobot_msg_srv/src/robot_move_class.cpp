#include <gobot_msg_srv/robot_move_class.h>

namespace robot_class {
    RobotMoveClass::RobotMoveClass(){}

    RobotMoveClass::~RobotMoveClass(){
        delete move_client_;
    }

    void RobotMoveClass::moveClientInitialize(){
        move_client_= new MoveBaseClient("move_base", true);
        move_client_->waitForServer(ros::Duration(60.0));
        //initilize set_robot_class subscriber
        initialize();
    }

    void RobotMoveClass::moveFromCS(){
        gobot_msg_srv::IsCharging isCharging;
        ros::service::call("/gobot_status/charging_status", isCharging);
        if(isCharging.response.isCharging){
            ROS_WARN("(ROBOT_MOVE_CLASS) Go straight because of charging.");
            forward(15);
            ros::Duration(2.5).sleep();
            stop();
            setDock(0);
        }
    }

    void RobotMoveClass::moveTo(Point point){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position.x = point.x;
        goal.target_pose.pose.position.y = point.y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(point.yaw);
        moveTo(goal);
    }

    void RobotMoveClass::moveTo(double x, double y, double yaw){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        moveTo(goal);
    }

    //tf::createQuaternionMsgFromYaw   //tf::getYaw (const geometry_msgs::Quaternion &msg_q)
    void RobotMoveClass::moveTo(double x, double y, geometry_msgs::Quaternion quternion){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation = quternion;
        moveTo(goal);
    }

    void RobotMoveClass::moveTo(move_base_msgs::MoveBaseGoal point){
        if(point.target_pose.header.frame_id.empty())
            point.target_pose.header.frame_id = "map";
        point.target_pose.header.stamp = ros::Time::now();

        if(move_client_->isServerConnected())
            move_client_->sendGoal(point);
    }

    void RobotMoveClass::cancelMove(){
        if(move_client_->isServerConnected())
            move_client_->cancelAllGoals();
    }

    actionlib::SimpleClientGoalState RobotMoveClass::getGoalState(){
        if(move_client_->isServerConnected())
            return move_client_->getState();
    }

    void RobotMoveClass::turnLeft(const int vel_r, const int vel_l){
        setNavSpeed('F', vel_r, 'B', vel_l);
    }

    void RobotMoveClass::turnRight(const int vel_r, const int vel_l){
        setNavSpeed('B', vel_r, 'F', vel_l);
    }

    void RobotMoveClass::turnLeft(const int vel){
        setNavSpeed('F', vel, 'B', vel);
    }

    void RobotMoveClass::turnRight(const int vel){
        setNavSpeed('B', vel, 'F', vel);
    }

    void RobotMoveClass::backward(const int vel){
        setNavSpeed('B', vel, 'B', vel);
    }

    void RobotMoveClass::forward(const int vel){
        setNavSpeed('F', vel, 'F', vel);
    }

    void RobotMoveClass::stop(){
        setNavSpeed('F', 0, 'F', 0);
    }

    void RobotMoveClass::getStatus(int &status, std::string &text){
        gobot_msg_srv::GetGobotStatus gobot_status;
        ros::service::call("/gobot_status/get_gobot_status",gobot_status);
        status = gobot_status.response.status;
        text = gobot_status.response.text;
    }

    void RobotMoveClass::setAutoSpeedLimit(double linear_v, double angular_v){
        dynamic_reconfigure::Reconfigure config;
        dynamic_reconfigure::DoubleParameter linear_param,angular_param;
        linear_param.name = "max_vel_x";
        linear_param.value = linear_v;
        angular_param.name = "max_vel_theta";
        angular_param.value = angular_v;
        config.request.config.doubles.push_back(linear_param);
        config.request.config.doubles.push_back(angular_param);
        ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config);
    }

    void RobotMoveClass::setManualSpeedLimit(double linear_v, double angular_v){
        gobot_msg_srv::SetFloatArray joy_speed;
        joy_speed.request.data.push_back(linear_v);
        joy_speed.request.data.push_back(angular_v);
        ros::service::call("/gobot_base/set_joy_speed",joy_speed);
    }
};