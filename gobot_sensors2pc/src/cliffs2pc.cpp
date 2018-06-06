#include <gobot_sensors2pc/cliffs2pc.hpp>

double CLIFF_THRESHOLD=170;

ros::Publisher cliffFRPublisher,cliffFLPublisher,cliffBRPublisher,cliffBLPublisher;
int left_speed_ = 0, right_speed_ = 0;

std::string front_left_cliff_frame, front_right_cliff_frame, back_left_cliff_frame, back_right_cliff_frame;
bool use_pc = true;

bool cliffToCloud(double CliffData,pcl::PointCloud<pcl::PointXYZ> &cloudData){
    if(CliffData>CLIFF_THRESHOLD){  
        cloudData.push_back(pcl::PointXYZ(0, 0, 0));
        return true;
    }
    else{
        return false;
    }
}

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
    left_speed_ = speed->velocityL;
    right_speed_ = speed->velocityR;
}

//see from front view/back view
//cliff1->front right, cliff2->front left, cliff3->back right, cliff4->back left
void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff){
    if(use_pc){
        bool cliff1_on, cliff2_on, cliff3_on, cliff4_on;
        pcl::PointCloud<pcl::PointXYZ> FRcliffCloud,FLcliffCloud,BRcliffCloud,BLcliffCloud;
        FRcliffCloud.header.frame_id = front_right_cliff_frame;
        FLcliffCloud.header.frame_id = front_left_cliff_frame;
        BRcliffCloud.header.frame_id = back_right_cliff_frame;
        BLcliffCloud.header.frame_id = back_left_cliff_frame;

        //if robot is moving
        if(left_speed_ != 0 || right_speed_ != 0){
            cliff1_on = cliffToCloud(cliff->cliff1,FRcliffCloud);
            cliff2_on = cliffToCloud(cliff->cliff2,FRcliffCloud);
            cliff3_on = cliffToCloud(cliff->cliff3,FRcliffCloud);
            cliff4_on = cliffToCloud(cliff->cliff4,FRcliffCloud);
        }

        cliffFRPublisher.publish(FRcliffCloud);
        cliffFLPublisher.publish(FLcliffCloud);
        cliffBRPublisher.publish(BRcliffCloud);
        cliffBLPublisher.publish(BLcliffCloud);
    }
}

void initParams(){
    ros::NodeHandle nh;

    /// We get the frames on which the cliffs are attached
    nh.param("front_right_cliff", front_right_cliff_frame, std::string("/front_right_cliff"));
    nh.param("front_left_cliff", front_left_cliff_frame, std::string("/front_left_cliff"));
    nh.param("back_right_cliff", back_right_cliff_frame, std::string("/back_right_cliff"));
    nh.param("back_left_cliff", back_left_cliff_frame, std::string("/back_left_cliff"));

    nh.getParam("CLIFF_THRESHOLD", CLIFF_THRESHOLD);
    nh.getParam("USE_CLIFF_PC", use_pc);
    std::cout << "(CLIFF2PC::initParams) CLIFF THRESHOLD:"<<CLIFF_THRESHOLD<<std::endl;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "cliffs2pc");
    ros::NodeHandle nh;
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    //Startup end

    initParams();
    cliffFRPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/FR_cliff_pc", 10);
    cliffFLPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/FL_cliff_pc", 10);
    cliffBRPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/BR_cliff_pc", 10);
    cliffBLPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/BL_cliff_pc", 10);
    // get the sonars information
    ros::Subscriber cliffSub = nh.subscribe("/gobot_base/cliff_topic", 1, cliffCallback);
    ros::Subscriber motorSpdSubscriber = nh.subscribe("/gobot_motor/motor_speed", 1, motorSpdCallback);

    ros::spin();

    return 0;
}