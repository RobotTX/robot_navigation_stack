#include <gobot_sensors2pc/cliffs2pc.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

double CLIFF_THRESHOLD=170, CLIFF_OUTRANGE=0;

ros::Publisher cliffFRPublisher,cliffFLPublisher,cliffBRPublisher,cliffBLPublisher;

std::string front_left_cliff_frame, front_right_cliff_frame, back_left_cliff_frame, back_right_cliff_frame;
bool FLcliff_on = false,FRcliff_on = false,BLcliff_on = false,BRcliff_on = false;

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    //ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR , velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("/gobot_motor/setSpeeds", speed);
}

bool cliffToCloud(double CliffData,pcl::PointCloud<pcl::PointXYZ> &cloudData, bool cliff_on){
    if(CliffData>CLIFF_THRESHOLD  || CliffData==CLIFF_OUTRANGE){
        cloudData.push_back(pcl::PointXYZ(0, 0, 0));
        return true;
    }
    else{
        return false;
    }
}

//see from front view/back view
//cliff1->front right, cliff2->front left, cliff3->back right, cliff4->back left
void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff){
    pcl::PointCloud<pcl::PointXYZ> FRcliffCloud,FLcliffCloud,BRcliffCloud,BLcliffCloud;
    FRcliffCloud.header.frame_id = front_right_cliff_frame;
    FLcliffCloud.header.frame_id = front_left_cliff_frame;
    BRcliffCloud.header.frame_id = back_right_cliff_frame;
    BLcliffCloud.header.frame_id = back_left_cliff_frame;
    gobot_msg_srv::GetInt get_speed;
    ros::service::call("/gobot_motor/getSpeeds",get_speed);
    //if robot is moving
    if(get_speed.response.data[0]!=128 || get_speed.response.data[1]!=128){
        if (cliffToCloud(cliff->cliff1,FRcliffCloud,FRcliff_on)){
            if(!FRcliff_on)
                FRcliff_on = true; 
        }
        else if(FRcliff_on){
            FRcliff_on = false;
        }

        if (cliffToCloud(cliff->cliff2,FLcliffCloud,FLcliff_on)){
            if(!FLcliff_on)
                FLcliff_on = true; 
        }
        else if(FLcliff_on){
            FLcliff_on = false;
        }

        if (cliffToCloud(cliff->cliff4,BRcliffCloud,BRcliff_on)){
            if(!BRcliff_on)
                BRcliff_on = true; 
        }
        else if(BRcliff_on){
            BRcliff_on = false;
        }

        if (cliffToCloud(cliff->cliff3,BLcliffCloud,BLcliff_on)){
            if(!BLcliff_on)
                BLcliff_on = true; 
        }
        else if(BLcliff_on){
            BLcliff_on = false;
        }
    }

    cliffFRPublisher.publish(FRcliffCloud);
    cliffFLPublisher.publish(FLcliffCloud);
    cliffBRPublisher.publish(BRcliffCloud);
    cliffBLPublisher.publish(BLcliffCloud);
}

bool initParams(){
    ros::NodeHandle nh;

    /// We get the frames on which the cliffs are attached
    nh.param("front_right_cliff", front_right_cliff_frame, std::string("/front_right_cliff"));
    nh.param("front_left_cliff", front_left_cliff_frame, std::string("/front_left_cliff"));
    nh.param("back_right_cliff", back_right_cliff_frame, std::string("/back_right_cliff"));
    nh.param("back_left_cliff", back_left_cliff_frame, std::string("/back_left_cliff"));

    nh.getParam("CLIFF_THRESHOLD", CLIFF_THRESHOLD);
    nh.getParam("CLIFF_OUTRANGE", CLIFF_OUTRANGE);
    std::cout << "(cliff2pc::initParams) CLIFF THRESHOLD:"<<CLIFF_THRESHOLD<<" CLIFF OUTRANGE:"<<CLIFF_OUTRANGE<<std::endl;


    return true;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "cliffs2pc");
    ros::NodeHandle nh;
    
    if(initParams()){
        cliffFRPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/FR_cliff_pc", 10);
        cliffFLPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/FL_cliff_pc", 10);
        cliffBRPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/BR_cliff_pc", 10);
        cliffBLPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/BL_cliff_pc", 10);
        // get the sonars information
        ros::Subscriber cliffSub = nh.subscribe("/gobot_base/cliff_topic", 1, cliffCallback);

        ros::spin();
    }

    return 0;
}