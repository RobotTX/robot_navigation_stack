#include <gobot_msg_srv/set_robot_class.h>


namespace robot_class {
    /**
    * @BRIEF  Class constructor
    */
    SetRobot::SetRobot(){};

    /**
    * @BRIEF  Class destructor
    */
    SetRobot::~SetRobot(){};

    //Do remember to initilize class after ros::init if setMotorSpeed is used
    void SetRobot::initialize(){
        ros::NodeHandle nh_;
        speed_pub_ = nh_.advertise<gobot_msg_srv::MotorSpeedMsg>("/gobot_motor/motor_speed", 1);
        led_pub_ = nh_.advertise<gobot_msg_srv::LedMsg>("/gobot_base/set_led", 1);
        sound_pub_ = nh_.advertise<gobot_msg_srv::SoundMsg>("/gobot_base/set_sound", 1);
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    }

    int SetRobot::stopRobotMoving(){
        gobot_msg_srv::GetGobotStatus get_gobot_status;
        ros::service::call("/gobot_status/get_gobot_status",get_gobot_status);

        //if robot is navigation / auto docking / scanning, then return the status
        if(get_gobot_status.response.status==5){
            ros::service::call("/gobot_command/pause_path",empty_srv);
            return get_gobot_status.response.status;
        }
        else if(get_gobot_status.response.status==15){
            ros::service::call("/gobot_command/stopGoDock",empty_srv);
            return get_gobot_status.response.status;
        }
        else if(get_gobot_status.response.status==25){
            ros::service::call("/gobot_command/stop_explore",empty_srv);
            return get_gobot_status.response.status;
        }
        else{
            //if robot is not moving, then return 0
            return 0;
        }
    }

    bool SetRobot::setStatus(int status,std::string text){
        set_gobot_status_.request.status = status;
        set_gobot_status_.request.text = text;
        return ros::service::call("/gobot_status/set_gobot_status",set_gobot_status_);
    }

    bool SetRobot::setDock(int status){
        set_dock_status_.request.data = status;
        return ros::service::call("/gobot_status/set_dock_status",set_dock_status_);
    }

    bool SetRobot::setStage(const int stage){
        set_stage_.request.data = stage;
        return ros::service::call("/gobot_status/set_stage", set_stage_);
    }

    bool SetRobot::setLoop(const int loop){
        set_loop_.request.data = loop;
        return ros::service::call("/gobot_status/set_loop",set_loop_);
    }

    bool SetRobot::setWifi(std::string wifi_name,std::string wifi_password){
        gobot_msg_srv::SetStringArray set_wifi;
        set_wifi.request.data.push_back(wifi_name);
        set_wifi.request.data.push_back(wifi_password);
        return ros::service::call("/gobot_status/set_wifi",set_wifi); 
    }  

    bool SetRobot::setMute(const int mute){
        gobot_msg_srv::SetInt set_mute;
        set_mute.request.data = mute;
        return ros::service::call("/gobot_status/set_mute", set_mute);
    }

    bool SetRobot::setName(std::string robot_name){
        gobot_msg_srv::SetString set_name;
        set_name.request.data = robot_name;
        return ros::service::call("/gobot_status/set_name",set_name);
    }

    bool SetRobot::setBatteryLvl(std::string battery_lvl){
        gobot_msg_srv::SetString set_battery;
        set_battery.request.data = battery_lvl;
        return ros::service::call("/gobot_status/set_battery",set_battery);
    }

    bool SetRobot::setSpeed(std::string linear, std::string angular){
        gobot_msg_srv::SetStringArray set_speed;
        set_speed.request.data.push_back(linear);
        set_speed.request.data.push_back(angular);
        return ros::service::call("/gobot_status/set_speed",set_speed);
    }

    bool SetRobot::setHome(std::string pos_x,std::string pos_y,std::string ori_x,std::string ori_y,std::string ori_z,std::string ori_w){
        gobot_msg_srv::SetStringArray set_home;
        set_home.request.data.push_back(pos_x);
        set_home.request.data.push_back(pos_y);
        set_home.request.data.push_back(ori_x);
        set_home.request.data.push_back(ori_y);
        set_home.request.data.push_back(ori_z);
        set_home.request.data.push_back(ori_w);
        return ros::service::call("/gobot_status/set_home",set_home);
    }

    bool SetRobot::clearPath(){
        gobot_msg_srv::SetStringArray set_path;
        ros::service::call("/gobot_status/set_path", set_path);
        return true;
    }

    //Do remember to initilize class after ros::init if setMotorSpeed is used
    bool SetRobot::setMotorSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){ 
        //maximum of int8 is 127
        motor_speed_.directionR = std::string(1, directionR);
        motor_speed_.velocityR = velocityR>127 ? 127 : velocityR;
        motor_speed_.directionL = std::string(1, directionL);
        motor_speed_.velocityL = velocityL>127 ? 127 : velocityL;
        speed_pub_.publish(motor_speed_);
        return true;
    }

    void SetRobot::setSound(int num,int time_on){
        gobot_msg_srv::SoundMsg soundCmd; 
        soundCmd.num = num;
        soundCmd.time_on = time_on;
        sound_pub_.publish(soundCmd);
    }
    
    void SetRobot::setInitialpose(const double p_x, const double p_y, const double q_x, const double q_y, const double q_z, const double q_w){
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.pose.pose.position.x = p_x;
        initial_pose.pose.pose.position.y = p_y;
        initial_pose.pose.pose.orientation.x = q_x;
        initial_pose.pose.pose.orientation.y = q_y;
        initial_pose.pose.pose.orientation.z = q_z;
        initial_pose.pose.pose.orientation.w = q_w;
        setInitialpose(initial_pose);
    }
            
    void SetRobot::setInitialpose(geometry_msgs::PoseWithCovarianceStamped pose){
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        //x-xy-y,yaw-yaw covariance
        pose.pose.covariance[0] = 0.01;
        pose.pose.covariance[7] = 0.01;
        pose.pose.covariance[35] = 0.01;
        if(pose.pose.pose.position.x == 0 && pose.pose.pose.position.y == 0 && pose.pose.pose.orientation.z == 0 && pose.pose.pose.orientation.w == 0){
            ROS_ERROR("(SET_ROBOT_CLASS) Assigned pose is invalid, set it position to map origin");
            pose.pose.pose.orientation.w = 1.0;
        }
        initial_pose_pub_.publish(pose);
    }

    void SetRobot::setBatteryLed(){
        gobot_msg_srv::LedMsg ledCmd; 
        ledCmd.mode = -1;
        led_pub_.publish(ledCmd);
    }

    void SetRobot::setLed(int mode, const std::vector<std::string> &color){
        gobot_msg_srv::LedMsg ledCmd; 
        ledCmd.mode = mode;
        for(int i = 0; i<color.size();i++)
            ledCmd.color.push_back(color[i]);
        led_pub_.publish(ledCmd);
    }

    std::string SetRobot::killList(){
        return "rosnode kill /move_base"; 
    }

    void SetRobot::reloadMap(){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        std::string cmd = "rosnode kill /map_server";
        system(cmd.c_str());
        ros::Duration(3.0).sleep();
        setStatus(-1,"ROBOT_READY");
        setBatteryLed();
    }


    void SetRobot::runNavi(bool simulation, bool reset_odom){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        if(reset_odom)
            ros::service::call("/gobot_motor/reset_odom",empty_srv);

        std::string cmd;
        ros::Duration(4.0).sleep();
        /// Relaunch gobot_navigation
       if(simulation)
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_slam.launch\"";
        else
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch >> ~/catkin_ws/src/robot_navigation_stack/robot_log/navigation_log.txt\"";
        system(cmd.c_str());
        ros::Duration(6.0).sleep();
    }

    void SetRobot::runScan(bool simulation, bool reset_odom){
        setMotorSpeed('F', 0, 'F', 0);
        setLed(0,{"white"});
        if(reset_odom)
            ros::service::call("/gobot_motor/reset_odom",empty_srv);
        
        std::string cmd;
        ros::Duration(4.0).sleep();
        /// Relaunch gobot_scan
        if(simulation)
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_gazebo gazebo_scan.launch\"";
        else
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_scan.launch >> ~/catkin_ws/src/robot_navigation_stack/robot_log/navigation_log.txt\"";
        system(cmd.c_str());
        ros::Duration(6.0).sleep();
    }

    //this two functions only work with robot equipped with speaker and ekho & festival packages
    void SetRobot::speakEnglish(std::string str){
        tts_en_ = "echo \"" + str + "\" | festival --tts &";
        system(tts_en_.c_str());
    }

    void SetRobot::speakChinese(std::string str){
        tts_ch_ = "ekho -s -30 \"" + str + "\" &";
        system(tts_ch_.c_str());
    }
    //this two functions only work with robot equipped with speaker and ekho & festival packages
    
};

