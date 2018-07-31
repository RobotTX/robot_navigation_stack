#include <gobot_base/sensor.hpp>

/*     
       B5 B6 B7 B8     
          IR_B
       C3       C4
       S3 P2 P1 S4
         __BB__
        |      |
  IR_R  |      |  IR_L
        |__FF__|
          LASER
        S2    S1
        C2    C1
       B4 B3 B2 B1
*/


int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_node");
    ros::NodeHandle n;
    
    std::cout<<"###############START SENSOR INITILIZATION###############"<<std::endl;
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(SENSOR::START) Waiting for MOTOR to be ready...");
    ros::service::waitForService("/gobot_startup/motor_ready", ros::Duration(60.0));
    ROS_INFO("(SENSOR::START) MOTOR is ready.");
    //Startup end

    SensorClass sensor_start;
    std::cout<<"###############COMPLETE SENSOR INITILIZATION###############"<<std::endl;
    ros::spin();

    return 0;
}