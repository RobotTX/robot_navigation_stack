#include <gobot_msg_srv/robot_command.h>


namespace robot_class {

    RobotCommand::RobotCommand(): global_costmap_(NULL), tf_(NULL), initialized_(false), 
    left_encoder_(0), right_encoder_(0), left_speed_(0), right_speed_(0), 
    battery_percent_(-1), charging_current_(-1), charging_flag_(false) {} 

    RobotCommand::~RobotCommand(){};

    //@Initialization
    void RobotCommand::initialize(){
        if(!initialized_){
            tf_ = new tf::TransformListener(ros::Duration(60));
            global_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", *tf_);
            set_robot_.initialize();
            
            ros::NodeHandle nh;
            //Publishers
            vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            //Subscribers
            encoder_sub_ = nh.subscribe<gobot_msg_srv::EncodersMsg>("/encoders", 50, &RobotCommand::encodersCallback, this);
            speed_sub_ = nh.subscribe<gobot_msg_srv::MotorSpeedMsg>("/gobot_motor/motor_speed", 1, &RobotCommand::motorSpdCallback,this);
            battery_sub_ = nh.subscribe<gobot_msg_srv::BatteryMsg>("/gobot_base/battery_topic", 1, &RobotCommand::batteryCallback,this);
            laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &RobotCommand::laserCallback,this);
            global_path_sub_ = nh.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 1, &RobotCommand::globalPathCallback,this);
            goal_sub_ = nh.subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1, &RobotCommand::goalCallback,this);
            sonar_sub_ = nh.subscribe<gobot_msg_srv::SonarMsg>("/gobot_base/sonar_topic", 1, &RobotCommand::sonarCallback,this);

            initialized_ = true;
        }
        else{
            ROS_WARN("(Robot_Command::You should not call initialize twice on this object, doing nothing");
        }
    }

    //@Subscriber callback
    void RobotCommand::encodersCallback(const gobot_msg_srv::EncodersMsg::ConstPtr& msg){
        left_encoder_ = msg->left_encoder;
        right_encoder_ = msg->right_encoder;
    }


    //motor speed ranges from 0 to 128. Positive indicates forward, and negative indicates backward
    void RobotCommand::motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
        if(speed->velocityL <= 127 && speed->velocityR <= 127){
             left_speed_ = speed->directionL.compare("F") == 0 ? -speed->velocityL : speed->velocityL;
             right_speed_ = speed->directionR.compare("F") == 0 ? -speed->velocityR : speed->velocityR;
        }
        else{
            left_speed_ = 128;
            right_speed_ = 128;
        } 
    }

    void RobotCommand::batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg){
        battery_percent_ = msg->BatteryStatus;
        charging_current_ = msg->ChargingCurrent;
        charging_flag_ = msg->ChargingFlag;
    }

    void RobotCommand::laserCallback(const sensor_msgs::LaserScan msg){
        laser_data_ = msg;
    }

    void RobotCommand::globalPathCallback(const nav_msgs::Path msg){
        global_path_ = msg;
    }

    void RobotCommand::goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
        current_goal_ = msg->goal.target_pose;
    }

    void RobotCommand::sonarCallback(const gobot_msg_srv::SonarMsg msg){
        sonar_data_ = msg;
    }


    //@Programmable voice
    void RobotCommand::speakEnglish(std::string str){
        set_robot_.speakEnglish(str);
    }

    void RobotCommand::speakChinese(std::string str){
        set_robot_.speakChinese(str);
    }
    

    //@Interact with base sensors such as color and sound
    void RobotCommand::setLed(int mode, const std::vector<std::string> &color){
        set_robot_.setLed(mode,color);
    }

    void RobotCommand::setSound(int num,int time_on){
        set_robot_.setSound(num,time_on);
    }

    int RobotCommand::getBatteryPercent(){
        return battery_percent_;
    }

    int RobotCommand::getBatteryChargingCurrent(){
        return charging_current_;
    }

    bool RobotCommand::getBatteryCharging(){
        return charging_flag_;
    }


    //@Interact with base motors such as speed and encoder
    void RobotCommand::setMotorSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){ 
        set_robot_.setMotorSpeed(directionR,velocityR,directionL,velocityL);
    }

    void RobotCommand::setMotorSpeed(const double linear_vel, const double angular_vel){
        geometry_msgs::Twist vel;
        vel.linear.x = linear_vel;
        vel.angular.z = angular_vel;
        vel_pub_.publish(vel);
    }

    void RobotCommand::setSpeedLimit(std::string linear, std::string angular){
        set_robot_.setSpeed(linear,angular);
    }

    void RobotCommand::getEncoder(int &left_encoder, int &right_encoder){
        left_encoder = left_encoder_;
        right_encoder = right_encoder_;   
    }

    void RobotCommand::getMotorSpeed(int &left_speed, int &right_speed){
        left_speed = left_speed_;
        right_speed = right_speed_;   
    }


    //@RPY - Quaternion conversion
    void RobotCommand::getYaw(double &yaw, const geometry_msgs::Quaternion &q){
        yaw = tf::getYaw(tf::Quaternion(q.x,q.y,q.z,q.w));
    }

    void RobotCommand::getQuaternion(geometry_msgs::Quaternion &q, const double &yaw){
        q = tf::createQuaternionMsgFromYaw(yaw);
    }

    
    //@Localization 
    void RobotCommand::getCurrentPose(Pose &pose){
        tf::Stamped<tf::Pose> global_pose;
        global_costmap_->getRobotPose(global_pose);
        geometry_msgs::PoseStamped temp_position;
        tf::poseStampedTFToMsg(global_pose, temp_position);
        pose.x = temp_position.pose.position.x;
        pose.y = temp_position.pose.position.y;
        getYaw(pose.theta,temp_position.pose.orientation);
    }

    void RobotCommand::getCurrentPose(geometry_msgs::PoseStamped &pose){
        tf::Stamped<tf::Pose> global_pose;
        global_costmap_->getRobotPose(global_pose);
        tf::poseStampedTFToMsg(global_pose, pose);
    }


    //@Map
    //rosrun map_server map_saver [-f mapname]
    void RobotCommand::saveMapTo(std::string path){
        std::string cmd;
        if (path == ""){
            cmd = "rosrun map_server map_saver &";
        }
        else{
            cmd = "rosrun map_server map_saver -f "+ path +" &";
        }
        system(cmd.c_str());
    }


    //@Target Points || Routes
    //get the point cost in costmap
    //costmap_2d::FREE_SPACE=0, costmap_2d::INSCRIBED_INFLATED_OBSTACLE=253, costmap_2d::LETHAL_OBSTACLE=254, costmap_2d::NO_INFORMATION=255
    int RobotCommand::getPointCost(const int point_x,const int point_y){
        unsigned int x,y;
        global_costmap_->getCostmap()->worldToMap(point_x,point_y,x,y); 	
        int cost = global_costmap_->getCostmap()->getCost(x,y);
        return cost;
    }
    //assign targeted point to robot, and play assigned point
    //First param = k, 2nd is point name, 3rd is x coordinate, 4th is y coordinate, 5th is orientation, 6th is home bool
    bool RobotCommand::setTargetPoint(const Pose &point){
        if (getPointCost(point.x,point.y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return false;

        gobot_msg_srv::SetStringArray msg;
        msg.request.data.push_back("robot_lib");
        msg.request.data.push_back(std::to_string(point.x));
        msg.request.data.push_back(std::to_string(point.y));
        msg.request.data.push_back(std::to_string(point.theta));
        msg.request.data.push_back("0");
        ros::service::call("/gobot_command/play_point",msg);
        return true;
    }

    bool RobotCommand::setTargetPoint(const geometry_msgs::PoseStamped &point){
        if (getPointCost(point.pose.position.x,point.pose.position.y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return false;

        gobot_msg_srv::SetStringArray msg;
        msg.request.data.push_back("robot_lib");
        msg.request.data.push_back(std::to_string(point.pose.position.x));
        msg.request.data.push_back(std::to_string(point.pose.position.y));
        double yaw;
        getYaw(yaw,point.pose.orientation);
        msg.request.data.push_back(std::to_string(yaw));
        msg.request.data.push_back("0");
        ros::service::call("/gobot_command/play_point",msg);
        return true;
    }

    //assign targeted path to robot
    /// First param = i, then the path name, then quadriplets of parameters to represent path points (path point name, posX, posY, waiting time,orientation) 
    bool RobotCommand::setTargetPath(const std::vector<Path> &path, std::string path_name){
        gobot_msg_srv::SetStringArray msg;
        msg.request.data.push_back(path_name);
        for(int i=0;i<path.size();i++){
            //if point is unreachable, return false
            if (getPointCost(path[i].x,path[i].y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                return false;
            msg.request.data.push_back(path[i].name);
            msg.request.data.push_back(std::to_string(path[i].x));
            msg.request.data.push_back(std::to_string(path[i].y));
            msg.request.data.push_back(std::to_string(path[i].waiting));
            msg.request.data.push_back(std::to_string(path[i].theta));
        }
        ros::service::call("/gobot_command/set_path",msg);
        return true;
    }

    //get the current goal information if have
    void RobotCommand::getCurrentGoal(geometry_msgs::PoseStamped &goal){
        goal = current_goal_;
    }

    //@Control robot motion (play, pause, stop)
    //play assigned path
    void RobotCommand::playTargetPath(){
        ros::service::call("/gobot_command/play_path",empty_srv_);
    }

    //pause robot from playing path/point
    void RobotCommand::pauseRobot(){
        ros::service::call("/gobot_command/pause_path",empty_srv_);
    }

    //stop robot from playing path/point
    void RobotCommand::stopRobot(){
        ros::service::call("/gobot_command/stop_path",empty_srv_);
    }

    //start robot going to charging station
    void RobotCommand::startDock(){
        ros::service::call("/gobot_command/goDock",empty_srv_);
    }

    //stop robot going to charging station
    void RobotCommand::stopDock(){
        ros::service::call("/gobot_command/stopGoDock",empty_srv_);
    }

    //poweroff robot
    void RobotCommand::shutDown(){
        ros::service::call("/gobot_command/shutdown",empty_srv_);
    }


    //@Obstacle Detection & Avoidance
    //get laser raw data (detection of obstacles depends on the laser range)
    void RobotCommand::getLaserData(sensor_msgs::LaserScan &data){
        data = laser_data_;
    }

    //get sonar raw data
    void RobotCommand::getSonarData(std::vector<int> &data){
        data.clear();
        data.push_back(sonar_data_.distance1);
        data.push_back(sonar_data_.distance2);
        data.push_back(sonar_data_.distance3);
        data.push_back(sonar_data_.distance4);
    }

    //get planned path from current pose to goal pose if have
    void RobotCommand::getPlanPath(std::vector<geometry_msgs::PoseStamped> &plan_path){
        plan_path.clear();
        for(int i=0;i<global_path_.poses.size();i++)
            plan_path.push_back(global_path_.poses[i]);
    }   

    //make plan for any two points in the map, and give a plan path if have
    void RobotCommand::makePlanPath(const geometry_msgs::PoseStamped &start,const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path){
        nav_msgs::GetPlan get_plan;
        get_plan.request.start = start;
        get_plan.request.goal = goal;
        get_plan.request.tolerance = 0.5;
        ros::service::call("/move_base/make_plan",get_plan);
        plan_path.clear();
        for(int i=0;i<get_plan.response.plan.poses.size();i++)
            plan_path.push_back(get_plan.response.plan.poses[i]);
    }

    void RobotCommand::makePlanPath(const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path){
        nav_msgs::GetPlan get_plan;
        get_plan.request.goal = goal;
        get_plan.request.tolerance = 0.5;
        ros::service::call("/move_base/make_plan",get_plan);
        plan_path.clear();
        for(int i=0;i<get_plan.response.plan.poses.size();i++)
            plan_path.push_back(get_plan.response.plan.poses[i]);
    }

    //clear costmap 
    void RobotCommand::clearCostMap(){
        ros::service::call("/move_base/clear_costmaps",empty_srv_);
    }
};

