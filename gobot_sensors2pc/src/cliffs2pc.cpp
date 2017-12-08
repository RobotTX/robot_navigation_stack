#include <gobot_sensors2pc/cliffs2pc.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#define CLIFF_THRESHOLD 500
#define CLIFF_RANGE 0.27

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

void cliffToCloud(double CliffData,pcl::PointCloud<pcl::PointXYZ> &cloudData){
        cloudData.push_back(pcl::PointXYZ(CliffData, 0, 0));
}

//see from front view/back view
//cliff1->front right, cliff2->front left, cliff3->back right, cliff4->back left
void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff){
    pcl::PointCloud<pcl::PointXYZ> FRcliffCloud,FLcliffCloud,BRcliffCloud,BLcliffCloud;

    if(cliff->cliff1 > CLIFF_THRESHOLD || cliff->cliff1==0){
        if(!FRcliff_on){
            FRcliff_on = true;
            FRcliffCloud.header.frame_id = front_right_cliff_frame;
            cliffToCloud(CLIFF_RANGE,FRcliffCloud);
            cliffFRPublisher.publish(FRcliffCloud);
            //ROS_INFO("Front Right cliff1 detects a height difference: %f", (cliff->cliff1-CLIFF_HEIGHT)/100.0);
        }
    }
    else if(FRcliff_on){
        FRcliff_on = false;
    }

    if(cliff->cliff2 > CLIFF_THRESHOLD){
        if(!FLcliff_on){
            FLcliff_on = true;
            FLcliffCloud.header.frame_id = front_left_cliff_frame;
            cliffToCloud(CLIFF_RANGE,FLcliffCloud);
            cliffFLPublisher.publish(FLcliffCloud);
            //ROS_INFO("Front Left cliff2 detects a height difference: %f", (cliff->cliff2-CLIFF_HEIGHT)/100.0);
        }
    }
    else if(FLcliff_on){
        FLcliff_on = false;
    }
    
    if(cliff->cliff3 > CLIFF_THRESHOLD){
        if(!BRcliff_on){
            BRcliff_on = true;
            BRcliffCloud.header.frame_id = back_right_cliff_frame;
            cliffToCloud(CLIFF_RANGE,BRcliffCloud);
            cliffBRPublisher.publish(BRcliffCloud);
            //ROS_INFO("Back Right cliff3 detects a height difference: %f", (cliff->cliff3-CLIFF_HEIGHT)/100.0);
        }
    }
    else if(BRcliff_on){
        BRcliff_on = false;
    }
    
    if(cliff->cliff4 > CLIFF_THRESHOLD){
        if(!BLcliff_on){
            BLcliff_on = true;
            BLcliffCloud.header.frame_id = back_left_cliff_frame;
            cliffToCloud(CLIFF_RANGE,BLcliffCloud);
            cliffBLPublisher.publish(BLcliffCloud);
            //ROS_INFO("Back Left cliff4 detects a height difference: %f", (cliff->cliff4-CLIFF_HEIGHT)/100.0);
        }
    }
    else if(BLcliff_on){
        BLcliff_on = false;
    }
}

bool initParams(void){

    ros::NodeHandle nh;

    /// We get the frames on which the sonars are attached
    nh.param("front_right_cliff", front_right_cliff_frame, std::string("/front_right_cliff"));
    nh.param("front_left_cliff", front_left_cliff_frame, std::string("/front_left_cliff"));
    nh.param("back_right_cliff", back_right_cliff_frame, std::string("/back_right_cliff"));
    nh.param("back_left_cliff", back_left_cliff_frame, std::string("/back_left_cliff"));

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