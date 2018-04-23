#include <gobot_msg_srv/robot_command.h>


namespace robot_class {

    /**
    * @BRIEF  Class constructor
    */
    RobotCommand::RobotCommand(): global_costmap_(NULL), tf_(NULL), initialized_(false), 
    left_encoder_(0), right_encoder_(0), left_speed_(0), right_speed_(0), 
    battery_percent_(-1), charging_current_(-1), charging_flag_(false),
    goal_status_(-1), map_received_(false), load_weight_(0) {} 


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
        map_sub_.shutdown();
        weight_sub_.shutdown();
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
            encoder_sub_ = nh.subscribe<gobot_msg_srv::EncodersMsg>("/encoders", 1, &RobotCommand::encodersCallback, this);
            speed_sub_ = nh.subscribe<gobot_msg_srv::MotorSpeedMsg>("/gobot_motor/motor_speed", 1, &RobotCommand::motorSpdCallback,this);
            battery_sub_ = nh.subscribe<gobot_msg_srv::BatteryMsg>("/gobot_base/battery_topic", 1, &RobotCommand::batteryCallback,this);
            laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &RobotCommand::laserCallback,this);
            global_path_sub_ = nh.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 1, &RobotCommand::globalPathCallback,this);
            goal_sub_ = nh.subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1, &RobotCommand::goalCallback,this);
            sonar_sub_ = nh.subscribe<gobot_msg_srv::SonarMsg>("/gobot_base/sonar_topic", 1, &RobotCommand::sonarCallback,this);
            gyro_sub_ = nh.subscribe<gobot_msg_srv::GyroMsg>("/gobot_base/gyro_topic", 1, &RobotCommand::gyroCallback,this);
            goal_status_sub_ = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, &RobotCommand::goalStatusCallback,this);
            map_sub_ = nh.subscribe<nav_msgs::MapMetaData>("/map_metadata", 1, &RobotCommand::mapDataCallback,this);
            weight_sub_ = nh.subscribe<gobot_msg_srv::WeightMsg>("/gobot_base/weight_topic", 1, &RobotCommand::weightCallback,this);

            ros::Rate r(5);
            while (!map_received_ && nh.ok()){
            ros::spinOnce();
            r.sleep();
            }
            initialized_ = true;
            ROS_INFO("(ROBOT_COMMAND) Object is successfully initialized.");
        }
        else{
            ROS_WARN("(ROBOT_COMMAND) You should not call initialize twice on this object, doing nothing");
        }
    }

    /**
    * @brief stops costmap updates and unsubscribes from sensor topics. 
    */
    void RobotCommand::stopCostmap(){
        if(initialized_){
            global_costmap_->stop();
            ROS_INFO("(ROBOT_COMMAND) Costmap update stopped.");
        }
        else{
            ROS_WARN("(ROBOT_COMMAND) You should initialize this object before calling this function");
        }
    }

    /**
    * @brief subscribes to sensor topics if necessary and starts costmap updates
    */
    void RobotCommand::startCostmap(){
        if(initialized_){
            global_costmap_->start();
            ROS_INFO("(ROBOT_COMMAND) Costmap update started.");
        }
        else{
            ROS_WARN("(ROBOT_COMMAND) You should initialize this object before calling this function");
        }
    }


    /**
    * @BRIEF  Subscribers callback
    */
    /**
    * @brief encoders callback function
    */
    void RobotCommand::encodersCallback(const gobot_msg_srv::EncodersMsg::ConstPtr& msg){
        left_encoder_ = msg->left_encoder;
        right_encoder_ = msg->right_encoder;
    }

    /**
    * @brief motor speeds callback function
    */
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

    /**
    * @brief battery callback function
    */
    void RobotCommand::batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg){
        battery_percent_ = msg->BatteryStatus;
        charging_current_ = msg->ChargingCurrent;
        charging_flag_ = msg->ChargingFlag;
    }

    /**
    * @brief laser callback function
    */
    void RobotCommand::laserCallback(const sensor_msgs::LaserScan msg){
        laser_data_ = msg;
    }

    /**
    * @brief planned path callback function
    */
    void RobotCommand::globalPathCallback(const nav_msgs::Path msg){
        global_path_ = msg;
    }

    /**
    * @brief goal callback function
    */
    void RobotCommand::goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
        ROS_INFO("(ROBOT_COMMAND) Received a new goal");
        current_goal_ = msg->goal.target_pose;
    }

    /**
    * @brief sonars callback function
    */
    void RobotCommand::sonarCallback(const gobot_msg_srv::SonarMsg msg){
        sonar_data_ = msg;
    }

    /**
    * @brief gyro and accelerometer callback function
    */
    void RobotCommand::gyroCallback(const gobot_msg_srv::GyroMsg::ConstPtr& msg){
        gyro_data_.gyrox = msg->gyrox;
        gyro_data_.gyroy = msg->gyroy;
        gyro_data_.gyroz = msg->gyroz;
        gyro_data_.accelx = msg->accelx;
        gyro_data_.accely = msg->accely;
        gyro_data_.accelz = msg->accelz;
    }

    /**
    * @brief goal status callback function
    */
    void RobotCommand::goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
        if(msg->status_list.size()==0)
            goal_status_ = -1;
        else
            goal_status_ = msg->status_list.back().status;
    }

    /**
    * @brief map metadata status callback function
    */
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
    * @brief load weight callback function
    */
    void RobotCommand::weightCallback(const gobot_msg_srv::WeightMsg::ConstPtr& msg){
        load_weight_ = msg->weight;
    }
    

    /**
    * @BRIEF  Some functions
    */
     /**
    * @brief quaternion to yaw conversion
    * @param yaw: get the yaw by converting given quaternion
    * @param q: set the quaternion for yaw convertion
    */
    void RobotCommand::getYaw(double &yaw, const geometry_msgs::Quaternion &q){
        yaw = tf::getYaw(tf::Quaternion(q.x,q.y,q.z,q.w));
    }

    /**
    * @brief yaw to quaternion conversion
    * @param q: get the quaternion by converting given yaw
    * @param yaw: set the yaw for quaternion convertion
    */
    void RobotCommand::getQuaternion(geometry_msgs::Quaternion &q, const double &yaw){
        q = tf::createQuaternionMsgFromYaw(yaw);
    }

    /**
    * @brief degree to radian conversion
    * @param degree: degree value that going to be converted to radian
    */
    double RobotCommand::getRadian(const double degree){
        return degree*3.1415926/180.0;
    }

    /**
    * @brief radian to yaw conversion
    * @param rad: radian value that going to be converted to degree
    */
    double RobotCommand::getDegree(const double rad){
        return rad*180.0/3.1415926;
    }


    /**
    * @BRIEF  Programmable voice
    * @param str: text for robot to convert to speech
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

    /**
    * @brief change robot led color
    * @param mode: 0 means permanent LED display; 1 means running LED display
    * @param color: LED colors to display (mode 0 needs only 1 color input, and mode 2 needs at least 2 colors input)
    * *color value: "green", "blue", "yellow", "red", "cyan", "white", "magenta", "off"
    */
    void RobotCommand::setLed(int mode, const std::vector<std::string> &color){
        set_robot_.setLed(mode,color);
    }

    /**
    * @brief make robot buzzer beep
    * @param num: how many times of beeps for the buzzer 
    * @param time_on: how long each beep last for
    */
    void RobotCommand::setSound(int num,int time_on){
        set_robot_.setSound(num,time_on);
    }


    /**
    * @brief return current battery percentage ranging from 0 to 100 (integer)
    */
    int RobotCommand::getBatteryPercent(){
        return battery_percent_;
    }

    /**
    * @brief return current battery charging current (integer); 
    * *usually positive values when charging, and negative values when not charging;
    */
    int RobotCommand::getBatteryChargingCurrent(){
        return charging_current_;
    }

    /**
    * @brief return current charging status; true indicates charging
    */
    bool RobotCommand::getBatteryCharging(){
        return charging_flag_;
    }


    /**
    * @brief get gyro data
    * @param gyro: get the current gyroscope and accelerometer data (self-defined type 'GyroMsg')
    */
    void RobotCommand::getGyro(Gyro &gyro_data){
        gyro_data = gyro_data_;
    }

    /**
    * @brief return load weight data (unit:kg)
    */
    float RobotCommand::getWeight(){
        return load_weight_;
    }

    /**
    * @brief turn on/off bumper sensors
    * @param flag: true/false means on/off bumper sensors
    */
    void RobotCommand::useBumper(bool flag){
        gobot_msg_srv::SetBool on_off;
        on_off.request.data = flag;
        ros::service::call("/gobot_base/use_bumper",on_off);
    }

    /**
    * @brief turn on/off sonar sensors
    * @param flag: true/false means on/off sonar sensors
    */
    void RobotCommand::useSonar(bool flag){
        gobot_msg_srv::SetBool on_off;
        on_off.request.data = flag;
        ros::service::call("/gobot_base/use_sonar",on_off);
    }

    /**
    * @brief turn on/off cliff sensors
    * @param flag: true/false means on/off cliff sensors
    */
    void RobotCommand::useCliff(bool flag){
        gobot_msg_srv::SetBool on_off;
        on_off.request.data = flag;
        ros::service::call("/gobot_base/use_cliff",on_off);
    }

    /**
    * @BRIEF  Interact with base motors such as get/set motor speeds and encoders
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
    void RobotCommand::setMotorSpeed(const char directionL, const int velocityL, const char directionR, const int velocityR){ 
        set_robot_.setMotorSpeed(directionR,velocityR,directionL,velocityL);
    }

    /**
    * @param linear_vel: set linear velocity of robot
    * @param angular_vel: set angular velocity of robot
    */
    void RobotCommand::setMotorSpeed(const double linear_vel, const double angular_vel){
        geometry_msgs::Twist vel;
        vel.linear.x = linear_vel;
        vel.angular.z = angular_vel;
        vel_pub_.publish(vel);
    }

    /**
    * @param linear: set limit for linear velocity with unit m/s (recommanded value: 0.4m/s)
    * @param angular: set limit for angular velocity with unit rad/s (recommanded value: 0.8 rad/s)
    */
    void RobotCommand::setSpeedLimit(double linear, double angular){
        gobot_msg_srv::SetStringArray set_speed;
        set_speed.request.data.push_back(std::to_string(linear));
        set_speed.request.data.push_back(std::to_string(getDegree(angular)));
        ros::service::call("/gobot_command/set_speed",set_speed);
    }

    /**
    * @param left_encoder: get left encoder's accumulated reading
    * @param right_encoder: get right encoder's accumulated reading
    */
    void RobotCommand::getEncoder(int &left_encoder, int &right_encoder){
        left_encoder = left_encoder_;
        right_encoder = right_encoder_;   
    }

    /**
    * @param left_speed: get left wheel's speed ranging from -127 to 127 (128 indicates out of range)
    * @param right_speed: get right wheel's speed ranging from -127 to 127 (128 indicates out of range)
    */
    void RobotCommand::getMotorSpeed(int &left_speed, int &right_speed){
        left_speed = left_speed_;
        right_speed = right_speed_;   
    }

    
    /**
    * @BRIEF  Localization
    */

    /**
    * @brief two ways to get robot current pose in the global frame:
    * *1.self-defined data type 'Pose'
    * *2.ROS-defined data type 'geometry_msgs::PoseStamped'
    */
    /**
    * @param pose: get the current pose as type 'Pose' 
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

    /**
    * @param pose: get the current pose as type 'geometry_msgs::PoseStamped'
    */
    void RobotCommand::getCurrentPose(geometry_msgs::PoseStamped &pose){
        tf::Stamped<tf::Pose> global_pose;
        if (global_costmap_->getRobotPose(global_pose)){
            tf::poseStampedTFToMsg(global_pose, pose);
        }
    }

    /**
    * @brief two ways to set robot current pose in the global frame:
    * *1.self-defined data type 'Pose'
    * *2.ROS-defined data type 'geometry_msgs::PoseWithCovarianceStamped'
    */
    /**
    * @param pose: set the current pose as type 'Pose' 
    */
    void RobotCommand::setInitialpose(Pose &initial_pose){
        geometry_msgs::Quaternion qua;
        getQuaternion(qua, initial_pose.theta);
        set_robot_.setInitialpose(initial_pose.x, initial_pose.y, qua.x, qua.y, qua.z, qua.w);
    }
    
    /**
    * @param pose: set the current pose as type 'geometry_msgs::PoseWithCovarianceStamped'
    */
    void RobotCommand::setInitialpose(geometry_msgs::PoseWithCovarianceStamped initial_pose){
        set_robot_.setInitialpose(initial_pose);
    }

    
    /**
    * @BRIEF  Map
    */

    /**
    * @param path: set the path to store robot's map to; if no path given, save map to /home/username/map.pgm
    */
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

    /**
    * @param data: get current map's data such as resolution, width, height and origin
    * *if map is not initialized, return false
    */
    void RobotCommand::getMapMetadata(Map &data){
        data = map_data_;
    }

    /**
    * @BRIEF  Target Points || Routes
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


    /**
    * @brief two ways to send a goal point for robot:
    * *1.self-defined data type 'Pose'
    * *2.ROS-defined data type 'geometry_msgs::PoseStamped'
    */
    /**
    * @param point: set the goal point as type 'Pose' 
    */
    bool RobotCommand::setTargetPoint(const Pose &point){
        if (getPointCost(point.x,point.y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            ROS_WARN("(ROBOT_COMMAND) Assigned point can not be reached.");
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

    /**
    * @param point: set the goal point as type 'geometry_msgs::PoseStamped' 
    */
    bool RobotCommand::setTargetPoint(const geometry_msgs::PoseStamped &point){
        if (getPointCost(point.pose.position.x,point.pose.position.y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            ROS_WARN("(ROBOT_COMMAND) Assigned point can not be reached.");
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

    /**
    * @brief set a route for robot
    * @param path: set the route as self-defined type 'Path' 
    * @param path_name: set the name for the route; if no name given, set route's name to be "default"
    */
    bool RobotCommand::setTargetPath(const std::vector<Path> &path, std::string path_name){
        gobot_msg_srv::SetStringArray msg;
        msg.request.data.push_back(path_name);
        for(int i=0;i<path.size();i++){
            //if point is unreachable, return false
            if (getPointCost(path[i].x,path[i].y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                ROS_WARN("(ROBOT_COMMAND) Assigned %d point of route can not be reached.", i);
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
            msg.request.data.push_back(path[i].text);
            msg.request.data.push_back(std::to_string(path[i].textDelay));
        }
        ros::service::call("/gobot_command/set_path",msg);
        return true;
    }

    /**
    * @brief get current goal of robot
    * @param goal: get the current goal information if have as type 'geometry_msgs::PoseStamped'
    */
    void RobotCommand::getCurrentGoal(geometry_msgs::PoseStamped &goal){
        goal = current_goal_;
    }

    /**
    * @brief return goal status
    * *-1 - No goal
    * * 1 - Goal active
    * * 2 - Goal cancel
    * * 3 - Goal complete
    * * 4 - Goal aborted
    */
    int RobotCommand::getGoalStatus(){
        return goal_status_;
    }



    /**
    * @BRIEF  Control robot motion
    */

    /**
    * @brief start robot to execute assigned route if have
    */
    void RobotCommand::playTargetPath(){
        ros::service::call("/gobot_command/play_path",empty_srv_);
    }

    /**
    * @brief stop robot; when re-executing route, robot will go to the uncompleted point
    */
    void RobotCommand::pauseRobot(){
        ros::service::call("/gobot_command/pause_path",empty_srv_);
    }

    /**
    * @brief stop robot; when re-executing route, robot will go to first point 
    */
    void RobotCommand::stopRobot(){
        ros::service::call("/gobot_command/stop_path",empty_srv_);
    }

    /**
    * @brief start robot to go to docking station
    */
    void RobotCommand::startDock(){
        ros::service::call("/gobot_command/goDock",empty_srv_);
    }

    /**
    * @brief stop robot from going to docking station
    */
    void RobotCommand::stopDock(){
        ros::service::call("/gobot_command/stopGoDock",empty_srv_);
    }

    /**
    * @brief poweroff robot and electrical system
    */
    void RobotCommand::shutDown(){
        ros::service::call("/gobot_command/shutdown",empty_srv_);
    }


    /**
    * @BRIEF  Obstacle Detection & Avoidance
    */

    /**
    * @brief get laser data
    * @param data: get the laser data as type 'sensor_msgs::LaserScan'
    */
    void RobotCommand::getLaserData(sensor_msgs::LaserScan &data){
        data = laser_data_;
    }

    /**
    * @brief get four sonars data (two in front and two in rear)
    * @param data: get the sonars data as type 'std::vector<int>', where four sonars data are stored in data[0], data[1], data[2] and data[3] respectively
    */
    void RobotCommand::getSonarData(std::vector<int> &data){
        data.clear();
        data.push_back(sonar_data_.distance1);
        data.push_back(sonar_data_.distance2);
        data.push_back(sonar_data_.distance3);
        data.push_back(sonar_data_.distance4);
    }

    /**
    * @brief get planned path to goal pose if have
    * @param plan_path: get the planned path consisting of 'geometry_msgs::PoseStamped' points
    */
    void RobotCommand::getPlanPath(std::vector<geometry_msgs::PoseStamped> &plan_path){
        plan_path.clear();
        for(int i=0;i<global_path_.poses.size();i++)
            plan_path.push_back(global_path_.poses[i]);
    }   

    /**
    * @brief plan path for given start pose and goal pose
    * *return true if have plan 
    * *return false if no plan
    * @param plan_path: get the planned path consisting of 'geometry_msgs::PoseStamped' points
    */
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

    /**
    * @brief get planned path from current pose to given goal pose if have
    * *return true if have plan 
    * *return false if no plan
    * @param plan_path: get the planned path consisting of 'geometry_msgs::PoseStamped' points
    */
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

    /**
    * @brief clear obstacles information in costmap. 
    * *As a result, only static map information will be kept after calling this function
    */
    void RobotCommand::clearCostMap(){
        ros::service::call("/move_base/clear_costmaps",empty_srv_);
    }
};

