#include <gobot_base/sensor.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_node");
    ros::NodeHandle n;
    
    std::cout<<"###############START SENSOR INITILIZATION###############"<<std::endl;
    
    //Startup begin
    ROS_INFO("(SENSOR::START) Waiting for MOTOR to be ready...");
    ros::service::waitForService("/gobot_startup/motor_ready", ros::Duration(60.0));
    ROS_INFO("(SENSOR::START) MOTOR is ready.");
    //Startup end

    SensorClass sensor_start;
    std::cout<<"###############COMPLETE SENSOR INITILIZATION###############"<<std::endl;
    ros::spin();

    return(0);
}