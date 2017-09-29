#include <gobot_sensors2pc/sonars2pc.hpp>

#define SONAR_MAX 150

ros::Publisher rearPublisher;
ros::Publisher frontLeftPublisher;
ros::Publisher frontRightPublisher;
ros::Publisher leftPublisher;
ros::Publisher rightPublisher;
ros::Publisher midPublisher;
ros::Publisher topPublisher;

std::string rear_frame, front_right_frame, front_left_frame, left_frame, right_frame, top_frame, mid_frame;


void newSonarsInfo(const gobot_msg_srv::SonarMsg::ConstPtr& sonars){
    pcl::PointCloud<pcl::PointXYZ> rearCloud;
    rearCloud.header.frame_id = rear_frame;
    rearCloud.push_back(pcl::PointXYZ(sonars->distance1 > SONAR_MAX ? 2.5 : sonars->distance1/100.0, 0, 0));
    rearPublisher.publish(rearCloud);

    pcl::PointCloud<pcl::PointXYZ> frontRight;
    frontRight.header.frame_id = front_right_frame;
    frontRight.push_back(pcl::PointXYZ(sonars->distance2 > SONAR_MAX ? 2.5 : sonars->distance2/100.0, 0, 0));
    frontRightPublisher.publish(frontRight);
    
    pcl::PointCloud<pcl::PointXYZ> frontLeft;
    frontLeft.header.frame_id = front_left_frame;
    frontLeft.push_back(pcl::PointXYZ(sonars->distance3 > SONAR_MAX ? 2.5 : sonars->distance3/100.0, 0,0 ));
    frontLeftPublisher.publish(frontLeft);
    
    pcl::PointCloud<pcl::PointXYZ> leftCloud;
    leftCloud.header.frame_id = left_frame;
    leftCloud.push_back(pcl::PointXYZ(sonars->distance4 > SONAR_MAX ? 2.5 : sonars->distance4/100.0, 0, 0));
    leftPublisher.publish(leftCloud);
    
    pcl::PointCloud<pcl::PointXYZ> rightCloud;
    rightCloud.header.frame_id = right_frame;
    rightCloud.push_back(pcl::PointXYZ(sonars->distance5 > SONAR_MAX ? 2.5 : sonars->distance5/100.0, 0, 0));
    rightPublisher.publish(rightCloud);
    
    pcl::PointCloud<pcl::PointXYZ> topCloud;
    topCloud.header.frame_id = top_frame;
    topCloud.push_back(pcl::PointXYZ(sonars->distance6 > SONAR_MAX ? 2.5 : sonars->distance6/100.0, 0, 0));
    topPublisher.publish(topCloud);
    
    pcl::PointCloud<pcl::PointXYZ> midCloud;
    midCloud.header.frame_id = mid_frame;
    midCloud.push_back(pcl::PointXYZ(sonars->distance7 > SONAR_MAX ? 2.5 : sonars->distance7/100.0, 0, 0));
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

            ros::spin();
        }

    } catch (std::exception& e) {
        std::cerr << "(sonars2pc) Exception: " << e.what() << std::endl;
    }

    return 0;
}