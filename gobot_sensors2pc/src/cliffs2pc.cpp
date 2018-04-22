#include <gobot_sensors2pc/cliffs2pc.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

double CLIFF_THRESHOLD=170, CLIFF_OUTRANGE=0;

ros::Publisher cliffFRPublisher,cliffFLPublisher,cliffBRPublisher,cliffBLPublisher;
int left_speed_ = 0, right_speed_ = 0;

std::string front_left_cliff_frame, front_right_cliff_frame, back_left_cliff_frame, back_right_cliff_frame;
bool FLcliff_on = false,FRcliff_on = false,BLcliff_on = false,BRcliff_on = false;
bool use_pc = true;

bool cliffToCloud(double CliffData,pcl::PointCloud<pcl::PointXYZ> &cloudData, bool cliff_on){
    if(CliffData>CLIFF_THRESHOLD){  //&& CliffData==CLIFF_OUTRANGE){   //CLIFF_OUTRANGE probably back wheel blocked cliff sensors
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
        pcl::PointCloud<pcl::PointXYZ> FRcliffCloud,FLcliffCloud,BRcliffCloud,BLcliffCloud;
        FRcliffCloud.header.frame_id = front_right_cliff_frame;
        FLcliffCloud.header.frame_id = front_left_cliff_frame;
        BRcliffCloud.header.frame_id = back_right_cliff_frame;
        BLcliffCloud.header.frame_id = back_left_cliff_frame;
        //if robot is moving
        if(left_speed_ != 0 || right_speed_ != 0){
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
}

void initParams(){
    ros::NodeHandle nh;

    /// We get the frames on which the cliffs are attached
    nh.param("front_right_cliff", front_right_cliff_frame, std::string("/front_right_cliff"));
    nh.param("front_left_cliff", front_left_cliff_frame, std::string("/front_left_cliff"));
    nh.param("back_right_cliff", back_right_cliff_frame, std::string("/back_right_cliff"));
    nh.param("back_left_cliff", back_left_cliff_frame, std::string("/back_left_cliff"));

    nh.getParam("CLIFF_THRESHOLD", CLIFF_THRESHOLD);
    nh.getParam("CLIFF_OUTRANGE", CLIFF_OUTRANGE);
    nh.getParam("USE_CLIFF_PC", use_pc);
    std::cout << "(CLIFF2PC::initParams) CLIFF THRESHOLD:"<<CLIFF_THRESHOLD<<" CLIFF OUTRANGE:"<<CLIFF_OUTRANGE<<std::endl;
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