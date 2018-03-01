#include <gobot_msg_srv/robot_command.h>


namespace robot_class {

    RobotCommand::RobotCommand(): global_costmap_(NULL), tf_(NULL), initialized_(false), left_encoder_(0), right_encoder_(0) {} 

    RobotCommand::~RobotCommand(){};

    void RobotCommand::initialize(tf::TransformListener* tf,costmap_2d::Costmap2DROS* global_costmap){
        if(!initialized_){
            tf_ = tf;
            global_costmap_ = global_costmap;
            ac = new MoveBaseClient("move_base", true);
            set_robot_.initialize();
            
            ros::NodeHandle nh;
            //Publishers
            vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            //Subscribers
            encoder_sub_ = nh.subscribe<gobot_msg_srv::EncodersMsg>("/encoders", 50, &RobotCommand::encodersCallback, this);
            
            initialized_ = true;
        }
        else{
            ROS_WARN("(Robot_Command::You should not call initialize twice on this object, doing nothing");
        }
    }

    //Subscriber callback
    void RobotCommand::encodersCallback(const gobot_msg_srv::EncodersMsg::ConstPtr& msg){
        left_encoder_ = msg->left_encoder;
        right_encoder_ = msg->right_encoder;
    }

    //programmable voice
    void RobotCommand::speakEnglish(std::string str){
        set_robot_.speakEnglish(str);
    }

    void RobotCommand::speakChinese(std::string str){
        set_robot_.speakChinese(str);
    }
    
    //Interact with base sensors such as color and sound
    void RobotCommand::setLed(int mode, const std::vector<std::string> &color){
        set_robot_.setLed(mode,color);
    }

    void RobotCommand::setSound(int num,int time_on){
        set_robot_.setSound(num,time_on);
    }

    //Interact with base motors such as speed and encoder
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

    //RPY - Quaternion conversion
    void RobotCommand::getYaw(double &yaw, const geometry_msgs::Quaternion &q){
        yaw = tf::getYaw(tf::Quaternion(q.x,q.y,q.z,q.w));
    }

    void RobotCommand::getQuaternion(geometry_msgs::Quaternion &q, const double &yaw){
        q = tf::createQuaternionMsgFromYaw(yaw);
    }

    
    //Localization 
    void RobotCommand::getCurrentPose(Pose &pose){
        tf::Stamped<tf::Pose> global_pose;
        global_costmap_->getRobotPose(global_pose);
        geometry_msgs::PoseStamped temp_position;
        tf::poseStampedTFToMsg(global_pose, temp_position);
        pose.x = temp_position.pose.position.x;
        pose.y = temp_position.pose.position.y;
        pose.qx = temp_position.pose.orientation.x;
        pose.qy = temp_position.pose.orientation.y;
        pose.qz = temp_position.pose.orientation.z;
        pose.qw = temp_position.pose.orientation.w;
        getYaw(pose.theta,temp_position.pose.orientation);
    }

    void RobotCommand::getCurrentPose(geometry_msgs::PoseStamped &pose){
        tf::Stamped<tf::Pose> global_pose;
        global_costmap_->getRobotPose(global_pose);
        tf::poseStampedTFToMsg(global_pose, pose);
    }

    //Map
    //rosrun map_server map_saver [-f mapname]
    void saveMapTo(std::string path){
        std::string cmd;
        if (path == ""){
            cmd = "rosrun map_server map_saver &";
        }
        else{
            cmd = "rosrun map_server map_saver -f "+ path +" &";
        }
        system(cmd.c_str());
    }

};

