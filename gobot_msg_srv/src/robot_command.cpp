#include <gobot_msg_srv/robot_command.h>


namespace robot_class {

    /**
    * @BRIEF  Class constructor
    */
    RobotCommand::RobotCommand(): global_costmap_(NULL), tf_(NULL), initialized_(false), 
    left_encoder_(0), right_encoder_(0), left_speed_(0), right_speed_(0), 
    battery_percent_(-1), charging_current_(-1), charging_flag_(false),
    goal_status_(-1), map_received_(false) {} 


    /**
    * @BRIEF  Class destructor
    */
    RobotCommand::~RobotCommand(){
        encoder_sub_.shutdown();
        speed_sub_.shutdown();
        battery_sub_.shutdown();
        laser_sub_.shutdown();
        global_path_sub_.shutdown();
        goal_sub_.shutdown();
        sonar_sub_.shutdown();
        gyro_sub_.shutdown();
        goal_status_sub_.shutdown();
        delete global_costmap_;
        delete tf_;
    }


    /**
    * @BRIEF  Initialize class object
    */
    void RobotCommand::initialize(){
        if(!initialized_){
            tf_ = new tf::TransformListener(ros::Duration(60));
            global_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", *tf_);
            set_robot_.initialize();
            
            ros::NodeHandle nh;
            //Publishers
            vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            make_plan_pub_ = nh.advertise<nav_msgs::Path>("/robot_lib/make_path_plan",1,true);
            //Subscribers
            encoder_sub_ = nh.subscribe<gobot_msg_srv::EncodersMsg>("/encoders", 50, &RobotCommand::encodersCallback, this);
            speed_sub_ = nh.subscribe<gobot_msg_srv::MotorSpeedMsg>("/gobot_motor/motor_speed", 1, &RobotCommand::motorSpdCallback,this);
            battery_sub_ = nh.subscribe<gobot_msg_srv::BatteryMsg>("/gobot_base/battery_topic", 1, &RobotCommand::batteryCallback,this);
            laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &RobotCommand::laserCallback,this);
            global_path_sub_ = nh.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 1, &RobotCommand::globalPathCallback,this);
            goal_sub_ = nh.subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1, &RobotCommand::goalCallback,this);
            sonar_sub_ = nh.subscribe<gobot_msg_srv::SonarMsg>("/gobot_base/sonar_topic", 1, &RobotCommand::sonarCallback,this);
            gyro_sub_ = nh.subscribe<gobot_msg_srv::GyroMsg>("/gobot_base/gyro_topic", 1, &RobotCommand::gyroCallback,this);
            goal_status_sub_ = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, &RobotCommand::goalStatusCallback,this);
            map_sub_ = nh.subscribe<nav_msgs::MapMetaData>("/map_metadata", 1, &RobotCommand::mapDataCallback,this);

            ros::Rate r(5);
            while (!map_received_ && nh.ok()){
            ros::spinOnce();
            r.sleep();
            }
            initialized_ = true;
            ROS_INFO("Robot_Command::Object is successfully initialized.");
        }
        else{
            ROS_WARN("Robot_Command::You should not call initialize twice on this object, doing nothing");
        }
    }

    /**
    * @brief stops costmap updates and unsubscribes from sensor topics. 
    */
    void RobotCommand::stopCostmap(){
        if(initialized_){
            global_costmap_->stop();
            ROS_INFO("Robot_Command::Costmap update stopped.");
        }
        else{
            ROS_WARN("Robot_Command::You should initialize this object before calling this function");
        }
    }

    /**
    * @brief subscribes to sensor topics if necessary and starts costmap updates
    */
    void RobotCommand::startCostmap(){
        if(initialized_){
            global_costmap_->start();
            ROS_INFO("Robot_Command::Costmap update started.");
        }
        else{
            ROS_WARN("Robot_Command::You should initialize this object before calling this function");
        }
    }


    /**
    * @BRIEF  Subscribers callback
    */
    void RobotCommand::encodersCallback(const gobot_msg_srv::EncodersMsg::ConstPtr& msg){
        left_encoder_ = msg->left_encoder;
        right_encoder_ = msg->right_encoder;
    }


    //motor speed ranges from 0 to 128. Positive indicates forward, and negative indicates backward
    void RobotCommand::motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
        if(speed->velocityL <= 127 && speed->velocityR <= 127){
             // compare returns 0 if strings are equal
             left_speed_ = speed->directionL.compare("F") == 0 ? speed->velocityL : -speed->velocityL;
             right_speed_ = speed->directionR.compare("F") == 0 ? speed->velocityR : -speed->velocityR;
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
        ROS_INFO("Robot_Lib::Received a new goal");
        current_goal_ = msg->goal.target_pose;
    }

    void RobotCommand::sonarCallback(const gobot_msg_srv::SonarMsg msg){
        sonar_data_ = msg;
    }

    void RobotCommand::gyroCallback(const gobot_msg_srv::GyroMsg::ConstPtr& msg){
        gyro_data_.gyrox = msg->gyrox;
        gyro_data_.gyroy = msg->gyroy;
        gyro_data_.gyroz = msg->gyroz;
        gyro_data_.accelx = msg->accelx;
        gyro_data_.accely = msg->accely;
        gyro_data_.accelz = msg->accelz;
    }

    void RobotCommand::goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
        if(msg->status_list.size()==0)
            goal_status_ = -1;
        else
            goal_status_ = msg->status_list.back().status;
    }

    void RobotCommand::mapDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg){
        map_data_.resolution = msg->resolution;
        map_data_.pixel_width = msg->width;
        map_data_.pixel_height = msg->height;
        map_data_.meter_width = map_data_.resolution * map_data_.pixel_width;
        map_data_.meter_height = map_data_.resolution * map_data_.pixel_height;
        map_data_.origin.x = msg->origin.position.x;
        map_data_.origin.y = msg->origin.position.y;
        getYaw(map_data_.origin.theta,msg->origin.orientation);
        map_received_ = true;
    }
    

    /**
    * @BRIEF  Some functions
    */
    void RobotCommand::getYaw(double &yaw, const geometry_msgs::Quaternion &q){
        yaw = tf::getYaw(tf::Quaternion(q.x,q.y,q.z,q.w));
    }

    void RobotCommand::getQuaternion(geometry_msgs::Quaternion &q, const double &yaw){
        q = tf::createQuaternionMsgFromYaw(yaw);
    }

    double RobotCommand::getRadian(const double degree){
        return degree*3.1415926/180.0;
    }

    double RobotCommand::getDegree(const double rad){
        return rad*180.0/3.1415926;
    }


    /**
    * @BRIEF  Programmable voice
    */
    void RobotCommand::ttsEnglish(std::string str){
        set_robot_.speakEnglish(str);
    }

    void RobotCommand::ttsChinese(std::string str){
        set_robot_.speakChinese(str);
    }
    

    /**
    * @BRIEF  Interact with base sensors such as color, sound and sensors data
    */
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

    void RobotCommand::getGyro(Gyro &gyro_data){
        gyro_data = gyro_data_;
    }


    /**
    * @BRIEF  Interact with base motors such as get/set motor speeds and encoders
    */
    void RobotCommand::setMotorSpeed(const char directionL, const int velocityL, const char directionR, const int velocityR){ 
        set_robot_.setMotorSpeed(directionR,velocityR,directionL,velocityL);
    }

    void RobotCommand::setMotorSpeed(const double linear_vel, const double angular_vel){
        geometry_msgs::Twist vel;
        vel.linear.x = linear_vel;
        vel.angular.z = angular_vel;
        vel_pub_.publish(vel);
    }

    void RobotCommand::setSpeedLimit(double linear, double angular){
        gobot_msg_srv::SetStringArray set_speed;
        set_speed.request.data.push_back(std::to_string(linear));
        set_speed.request.data.push_back(std::to_string(getDegree(angular)));
        ros::service::call("/gobot_command/set_speed",set_speed);
    }

    void RobotCommand::getEncoder(int &left_encoder, int &right_encoder){
        left_encoder = left_encoder_;
        right_encoder = right_encoder_;   
    }

    void RobotCommand::getMotorSpeed(int &left_speed, int &right_speed){
        left_speed = left_speed_;
        right_speed = right_speed_;   
    }

    
    /**
    * @BRIEF  Localization
    */
    void RobotCommand::getCurrentPose(Pose &pose){
        tf::Stamped<tf::Pose> global_pose;
        if (global_costmap_->getRobotPose(global_pose)){
            geometry_msgs::PoseStamped temp_position;
            tf::poseStampedTFToMsg(global_pose, temp_position);
            pose.x = temp_position.pose.position.x;
            pose.y = temp_position.pose.position.y;
            getYaw(pose.theta,temp_position.pose.orientation);
        }
    }

    void RobotCommand::getCurrentPose(geometry_msgs::PoseStamped &pose){
        tf::Stamped<tf::Pose> global_pose;
        if (global_costmap_->getRobotPose(global_pose)){
            tf::poseStampedTFToMsg(global_pose, pose);
        }
    }


    /**
    * @BRIEF  Map
    */
    //rosrun map_server map_saver [-f mapname]
    void RobotCommand::saveMapTo(std::string path){
        std::string cmd;
        if (path == ""){
            cmd = "rosrun map_server map_saver -f ~/map &";
        }
        else{
            cmd = "rosrun map_server map_saver -f "+ path +" &";
        }
        system(cmd.c_str());
    }

    void RobotCommand::getMapMetadata(Map &data){
        data = map_data_;
    }

    /**
    * @BRIEF  Target Points || Routes
    */
    //get the point cost in costmap
    //costmap_2d::FREE_SPACE=0, costmap_2d::INSCRIBED_INFLATED_OBSTACLE=253, costmap_2d::LETHAL_OBSTACLE=254, costmap_2d::NO_INFORMATION=255
    int RobotCommand::getPointCost(const double point_x,const double point_y){
        unsigned int x,y;
        if(global_costmap_->getCostmap()->worldToMap(point_x,point_y,x,y)){	
            int cost = global_costmap_->getCostmap()->getCost(x,y);
            return cost;
        }
        else{
            return 255;
        }
    }
    //assign targeted point to robot, and play assigned point
    //First param = k, 2nd is point name, 3rd is x coordinate, 4th is y coordinate, 5th is orientation, 6th is home bool
    bool RobotCommand::setTargetPoint(const Pose &point){
        if (getPointCost(point.x,point.y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            ROS_WARN("Robot_Lib::Assigned point can not be reached.");
            return false;
        }

        gobot_msg_srv::SetStringArray msg;
        msg.request.data.push_back("robot_lib");
        msg.request.data.push_back(std::to_string(point.x));
        msg.request.data.push_back(std::to_string(point.y));
        //rads
        double theta = getDegree(-point.theta-1.57079);
        msg.request.data.push_back(std::to_string(theta));
        msg.request.data.push_back("0");
        ros::service::call("/gobot_command/play_point",msg);
        return true;
    }

    bool RobotCommand::setTargetPoint(const geometry_msgs::PoseStamped &point){
        if (getPointCost(point.pose.position.x,point.pose.position.y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            ROS_WARN("Robot_Lib::Assigned point can not be reached.");
            return false;
        }

        gobot_msg_srv::SetStringArray msg;
        msg.request.data.push_back("robot_lib");
        msg.request.data.push_back(std::to_string(point.pose.position.x));
        msg.request.data.push_back(std::to_string(point.pose.position.y));
        double yaw;
        getYaw(yaw,point.pose.orientation);
        yaw = getDegree(-yaw-1.57079);
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
            if (getPointCost(path[i].x,path[i].y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                ROS_WARN("Robot_Lib::Assigned %d point of route can not be reached.", i);
                return false;
            }
            std::string name = path[i].name == "" ? std::to_string(i+1) : path[i].name;
            msg.request.data.push_back(name);
            msg.request.data.push_back(std::to_string(path[i].x));
            msg.request.data.push_back(std::to_string(path[i].y));
            msg.request.data.push_back(std::to_string(path[i].waiting));
            //rads
            double theta = getDegree(-path[i].theta-1.57079);
            msg.request.data.push_back(std::to_string(theta));
        }
        ros::service::call("/gobot_command/set_path",msg);
        return true;
    }

    //get the current goal information if have
    void RobotCommand::getCurrentGoal(geometry_msgs::PoseStamped &goal){
        goal = current_goal_;
    }

    //get goal status
    int RobotCommand::getGoalStatus(){
        return goal_status_;
    }


    /**
    * @BRIEF  Control robot motion
    */
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


    /**
    * @BRIEF  Obstacle Detection & Avoidance
    */
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
    bool RobotCommand::makePlanPath(const geometry_msgs::PoseStamped &start,const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path){
        nav_msgs::GetPlan get_plan;
        get_plan.request.start = start;
        get_plan.request.goal = goal;
        get_plan.request.tolerance = 0.05;
        bool gotPlan = ros::service::call("/move_base/make_plan",get_plan);
        plan_path.clear();
        if(!gotPlan)
            return false;
        for(int i=0;i<get_plan.response.plan.poses.size();i++)
            plan_path.push_back(get_plan.response.plan.poses[i]);
        if(start.pose.position.x!=goal.pose.position.x || start.pose.position.y!=goal.pose.position.y 
        || start.pose.orientation.z!=goal.pose.orientation.z || start.pose.orientation.w!=goal.pose.orientation.w){
            if(plan_path.size()!=0)
                return true;
            else
                return false;
        }
        else{
            return true;
        }
    }

    bool RobotCommand::makePlanPath(const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan_path){
        nav_msgs::GetPlan get_plan;
        get_plan.request.goal = goal;
        get_plan.request.tolerance = 0.05;
        bool gotPlan = ros::service::call("/move_base/make_plan",get_plan);
        plan_path.clear();
        if(!gotPlan)
            return false;
        for(int i=0;i<get_plan.response.plan.poses.size();i++)
            plan_path.push_back(get_plan.response.plan.poses[i]);

        if(plan_path.size()!=0)
            return true;
        else
            return false;
    }

    //clear costmap 
    void RobotCommand::clearCostMap(){
        ros::service::call("/move_base/clear_costmaps",empty_srv_);
    }
};

