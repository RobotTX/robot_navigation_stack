#include <gobot_base/gyro.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "gyro_node");
    ros::NodeHandle n;
    
    std::cout<<"###############START GYRO INITILIZATION###############"<<std::endl;
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(GYRO::START) Waiting for SENSOR to be ready...");
    ros::service::waitForService("/gobot_startup/sensors_ready", ros::Duration(60.0));
    ROS_INFO("(GYRO::START) SENSOR is ready.");
    //Startup end

    GyroClass gyro_start;
    std::cout<<"###############COMPLETE GYRO INITILIZATION###############"<<std::endl;
    ros::spin();

    return 0;
}