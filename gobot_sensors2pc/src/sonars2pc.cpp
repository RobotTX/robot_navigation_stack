#include <gobot_sensors2pc/sonars2pc.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#define SONAR_THRESHOLD 1.5
#define SONAR_MAX 2.0
#define SONAR_VIEW 0.2 //11.5degree //0.165 //9.45degree 
#define SONAR_RESOLUTION 0.0165 //0.945 degree
#define SOME_FACTOR 0.25

ros::Publisher rearPublisher;
ros::Publisher frontLeftPublisher;
ros::Publisher frontRightPublisher;
ros::Publisher leftPublisher;
ros::Publisher rightPublisher;
ros::Publisher midPublisher;
ros::Publisher topPublisher;

std::string rear_frame, front_right_frame, front_left_frame, left_frame, right_frame, top_frame, mid_frame;
double clear_radius = 0.0;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    tf::TransformListener tf_listener;

}


void sonarToCloud(double sonarData,pcl::PointCloud<pcl::PointXYZ> &cloudData){
    sonarData = sonarData>SONAR_THRESHOLD?SONAR_MAX:sonarData;
    for(double i=-tan(SONAR_VIEW)*sonarData;i<=tan(SONAR_VIEW)*sonarData;i=i+SONAR_RESOLUTION)
        cloudData.push_back(pcl::PointXYZ(SONAR_MAX, i, 0));

    cloudData.push_back(pcl::PointXYZ(sonarData, 0, 0));
}

void sonarFrontToCloud(double sonarR,double sonarL,pcl::PointCloud<pcl::PointXYZ> &cloudR,pcl::PointCloud<pcl::PointXYZ> &cloudL){
    sonarR = sonarR>SONAR_THRESHOLD?SONAR_MAX:sonarR;
    sonarL = sonarL>SONAR_THRESHOLD?SONAR_MAX:sonarL;

    if(std::abs(sonarR-sonarL)<0.04){
        //Mark obstacle in the inner side
        for(double i=-tan(SONAR_VIEW)*sonarR;i<tan(SONAR_VIEW)*sonarR;i=i+SONAR_RESOLUTION)
            cloudR.push_back(pcl::PointXYZ(SONAR_MAX, i, 0)); 
        for(double i=tan(SONAR_VIEW)*sonarL;i>-tan(SONAR_VIEW)*sonarL;i=i-SONAR_RESOLUTION)
            cloudL.push_back(pcl::PointXYZ(SONAR_MAX, i, 0));
        cloudR.push_back(pcl::PointXYZ(sonarR, tan(SONAR_VIEW)*sonarR, 0));
        cloudL.push_back(pcl::PointXYZ(sonarL, -tan(SONAR_VIEW)*sonarL, 0));
    }
    else if(std::abs(sonarR-sonarL)<0.16){
        if(sonarR>sonarL){
            //Mark obstacles in the left side
            for(double i=-tan(SONAR_VIEW)*sonarR;i<tan(SONAR_VIEW)*sonarR;i=i+SONAR_RESOLUTION)
                cloudR.push_back(pcl::PointXYZ(SONAR_MAX, i, 0)); 
            for(double i=-tan(SONAR_VIEW)*sonarL;i<tan(SONAR_VIEW)*sonarL;i=i+SONAR_RESOLUTION)
                cloudL.push_back(pcl::PointXYZ(SONAR_MAX, i, 0));
            cloudR.push_back(pcl::PointXYZ(sonarR, tan(SONAR_VIEW)*sonarR, 0));
            cloudL.push_back(pcl::PointXYZ(sonarL, tan(SONAR_VIEW)*sonarL, 0));
        }
        else if(sonarR<sonarL){
            //Mark obstacles in the right side
            for(double i=tan(SONAR_VIEW)*sonarR;i>-tan(SONAR_VIEW)*sonarR;i=i-SONAR_RESOLUTION)
                cloudR.push_back(pcl::PointXYZ(SONAR_MAX, i, 0)); 
            for(double i=tan(SONAR_VIEW)*sonarL;i>-tan(SONAR_VIEW)*sonarL;i=i-SONAR_RESOLUTION)
                cloudL.push_back(pcl::PointXYZ(SONAR_MAX, i, 0));
            cloudR.push_back(pcl::PointXYZ(sonarR, -tan(SONAR_VIEW)*sonarR, 0));
            cloudL.push_back(pcl::PointXYZ(sonarL, -tan(SONAR_VIEW)*sonarL, 0));
        }
    }
    else{
        //Mark obstacle in the outter side
        for(double i=tan(SONAR_VIEW)*sonarR;i>-tan(SONAR_VIEW)*sonarR;i=i-SONAR_RESOLUTION)
            cloudR.push_back(pcl::PointXYZ(SONAR_MAX, i, 0)); 
        for(double i=-tan(SONAR_VIEW)*sonarL;i<tan(SONAR_VIEW)*sonarL;i=i+SONAR_RESOLUTION)
            cloudL.push_back(pcl::PointXYZ(SONAR_MAX, i, 0));
        cloudR.push_back(pcl::PointXYZ(sonarR, -tan(SONAR_VIEW)*sonarR, 0));
        cloudL.push_back(pcl::PointXYZ(sonarL, tan(SONAR_VIEW)*sonarL, 0));
    }
}


void newSonarsInfo(const gobot_msg_srv::SonarMsg::ConstPtr& sonars){
    pcl::PointCloud<pcl::PointXYZ> rearCloud,frontRightCloud,frontLeftCloud,leftCloud,rightCloud,topCloud,midCloud;
    
    rearCloud.header.frame_id = rear_frame;
    sonarToCloud(sonars->distance1/100.0,rearCloud);
    rearPublisher.publish(rearCloud);

    sonarFrontToCloud(sonars->distance2/100.0,sonars->distance3/100.0,frontRightCloud,frontLeftCloud);
    frontRightCloud.header.frame_id = front_right_frame;
    //sonarToCloud(sonars->distance2/100.0,frontRight);
    frontRightPublisher.publish(frontRightCloud);
    
    frontLeftCloud.header.frame_id = front_left_frame;
    //sonarToCloud(sonars->distance3/100.0,frontLeft);
    frontLeftPublisher.publish(frontLeftCloud);
    
    leftCloud.header.frame_id = left_frame;
    sonarToCloud(sonars->distance4/100.0,leftCloud);
    leftPublisher.publish(leftCloud);
    
    rightCloud.header.frame_id = right_frame;
    sonarToCloud(sonars->distance5/100.0,rightCloud);
    rightPublisher.publish(rightCloud);
    
    topCloud.header.frame_id = top_frame;
    sonarToCloud(sonars->distance6/100.0,topCloud);
    topPublisher.publish(topCloud);
    
    midCloud.header.frame_id = mid_frame;
    sonarToCloud(sonars->distance7/100.0,midCloud);
    midPublisher.publish(midCloud);
}

bool initParams(void){

    ros::NodeHandle nh("sonars2pc");

    /// We get the frames on which the sonars are attached
    nh.param("rear_frame", rear_frame, std::string("/rear_sonar"));
    nh.param("front_right_frame", front_right_frame, std::string("/front_right_sonar"));
    nh.param("front_left_frame", front_left_frame, std::string("/front_left_sonar"));
    nh.param("left_frame", left_frame, std::string("/left_sonar"));
    nh.param("right_frame", right_frame, std::string("/right_sonar"));
    nh.param("top_frame", top_frame, std::string("/top_sonar"));
    nh.param("mid_frame", mid_frame, std::string("/mid_sonar"));

    return true;
}


int main(int argc, char* argv[]){

    std::cout << "(sonars2pc) running..." << std::endl;

    try {

        ros::init(argc, argv, "sonars2pc");

        ros::NodeHandle nh("sonars2pc");

        if(initParams()){

            rearPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/rear_sonar_pc", 10);
            frontLeftPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/frontLeft_sonar_pc", 10);
            frontRightPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/frontRight_sonar_pc", 10);
            leftPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/left_sonar_pc", 10);
            rightPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/right_sonar_pc", 10);
            midPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/mid_sonar_pc", 10);
            topPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/top_sonar_pc", 10);

            // get the sonars information
            ros::Subscriber sonarSub = nh.subscribe("/sonar_topic", 1, newSonarsInfo);
            ros::Subscriber laserSub = nh.subscribe("/scan", 1, laserCallback);

            ros::spin();
        }

    } catch (std::exception& e) {
        std::cerr << "(sonars2pc) Exception: " << e.what() << std::endl;
    }

    return 0;
}