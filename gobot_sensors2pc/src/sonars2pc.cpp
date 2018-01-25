#include <gobot_sensors2pc/sonars2pc.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher rearRightPublisher,rearLeftPublisher;
ros::Publisher frontLeftPublisher,frontRightPublisher;
ros::Publisher leftPublisher;
ros::Publisher rightPublisher;
ros::Publisher midPublisher;
ros::Publisher topPublisher;

std::string rear_right_frame, rear_left_frame, front_right_frame, front_left_frame, left_frame, right_frame, top_frame, mid_frame;
double SONAR_THRESHOLD=1.0, SONAR_OUTRANGE=0, SONAR_MAX=1.5, SONAR_VIEW=0.2, SONAR_RESOLUTION=50.0;

void sonarToCloud(double sonarData,pcl::PointCloud<pcl::PointXYZ> &cloudData){
    sonarData = sonarData>SONAR_THRESHOLD?SONAR_MAX:sonarData;
    for(double i=-tan(SONAR_VIEW)*sonarData;i<=tan(SONAR_VIEW)*sonarData;i=i+SONAR_RESOLUTION)
        cloudData.push_back(pcl::PointXYZ(SONAR_MAX, i, 0));

    cloudData.push_back(pcl::PointXYZ(sonarData, 0, 0));
}

void sonarFrontToCloud(double sonarR,double sonarL,pcl::PointCloud<pcl::PointXYZ> &cloudR,pcl::PointCloud<pcl::PointXYZ> &cloudL, double y, double factor){

    sonarR=(sonarR==SONAR_OUTRANGE || sonarR>SONAR_THRESHOLD) ? SONAR_MAX : sonarR;
    sonarL=(sonarL==SONAR_OUTRANGE || sonarL>SONAR_THRESHOLD) ? SONAR_MAX : sonarL;

    for(double i=factor*SONAR_VIEW;i>-y;i=i-SONAR_VIEW/SONAR_RESOLUTION)
        cloudR.push_back(pcl::PointXYZ(SONAR_MAX*factor, i, 0));

    for(double i=-factor*SONAR_VIEW;i<y;i=i+SONAR_VIEW/SONAR_RESOLUTION)
        cloudL.push_back(pcl::PointXYZ(SONAR_MAX*factor, i, 0));

    if(fabs(sonarR-sonarL)>0.1 && (sonarR>0.5 || sonarL>0.5)){
        cloudR.push_back(pcl::PointXYZ(sonarR, 0, 0));
        cloudL.push_back(pcl::PointXYZ(sonarL, 0, 0));
    }
    else if ((sonarR>sonarL) && (sonarL!=SONAR_MAX)){
        cloudL.push_back(pcl::PointXYZ(sonarL, 0, 0));
    }
    else if(sonarR!=SONAR_MAX){
        cloudR.push_back(pcl::PointXYZ(sonarR, 0, 0));
    }
}


void newSonarsInfo(const gobot_msg_srv::SonarMsg::ConstPtr& sonars){
    pcl::PointCloud<pcl::PointXYZ> rearRightCloud,rearLeftCloud,frontRightCloud,frontLeftCloud;
    
    frontRightCloud.header.frame_id = front_right_frame;
    frontLeftCloud.header.frame_id = front_left_frame;
    rearRightCloud.header.frame_id = rear_right_frame;
    rearLeftCloud.header.frame_id = rear_left_frame;
/*
    sonarToCloud(sonars->distance1/100.0,frontRightCloud);
    sonarToCloud(sonars->distance2/100.0,frontLeftCloud);
    sonarToCloud(sonars->distance4/100.0,rearRightCloud);
    sonarToCloud(sonars->distance3/100.0,rearLeftCloud);
*/
    sonarFrontToCloud(sonars->distance1/100.0,sonars->distance2/100.0,frontRightCloud,frontLeftCloud,0.12,1);
    sonarFrontToCloud(sonars->distance3/100.0,sonars->distance4/100.0,rearLeftCloud,rearRightCloud,0.11,2);

    frontRightPublisher.publish(frontRightCloud);
    frontLeftPublisher.publish(frontLeftCloud);
    rearRightPublisher.publish(rearRightCloud);
    rearLeftPublisher.publish(rearLeftCloud);
}

bool initParams(){
    ros::NodeHandle nh;

    /// We get the frames on which the sonars are attached
    nh.param("rear_right_frame", rear_right_frame, std::string("/rear_right_sonar"));
    nh.param("rear_left_frame", rear_left_frame, std::string("/rear_left_sonar"));
    nh.param("front_right_frame", front_right_frame, std::string("/front_right_sonar"));
    nh.param("front_left_frame", front_left_frame, std::string("/front_left_sonar"));
    nh.param("top_frame", top_frame, std::string("/top_sonar"));
    nh.param("mid_frame", mid_frame, std::string("/mid_sonar"));

    nh.getParam("SONAR_THRESHOLD", SONAR_THRESHOLD);
    nh.getParam("SONAR_OUTRANGE", SONAR_OUTRANGE);
    nh.getParam("SONAR_MAX", SONAR_MAX);
    nh.getParam("SONAR_VIEW", SONAR_VIEW);
    nh.getParam("SONAR_RESOLUTION", SONAR_RESOLUTION);
    std::cout << "(sonar2pc::initParams) SONAR THRESHOLD:"<<SONAR_THRESHOLD<<" SONAR OUTRANGE:"<<SONAR_OUTRANGE<<
    " SONAR MAX:"<<SONAR_MAX<<" SONAR VIEW:"<<SONAR_VIEW<<" SONAR RESOLUTION:"<<SONAR_RESOLUTION<<std::endl;
    return true;
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "sonars2pc");
    ros::NodeHandle nh;

    if(initParams()){

        rearRightPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/rearRight_sonar_pc", 10);
        rearLeftPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/rearLeft_sonar_pc", 10);
        frontLeftPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/frontLeft_sonar_pc", 10);
        frontRightPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/frontRight_sonar_pc", 10);
        leftPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/left_sonar_pc", 10);
        rightPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/right_sonar_pc", 10);
        midPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/mid_sonar_pc", 10);
        topPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/top_sonar_pc", 10);

        // get the sonars information
        ros::Subscriber sonarSub = nh.subscribe("/gobot_base/sonar_topic", 1, newSonarsInfo);

        ros::spin();
    }

    return 0;
}