#include <gobot_base/motor.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle n;

  std::cout<<"###############START MOTOR INITILIZATION###############"<<std::endl;
  
  //Startup begin
  ROS_INFO("(MOTOR::START) Waiting for STATUS to be ready...");
  ros::service::waitForService("/gobot_status/set_gobot_status", ros::Duration(60.0));
  ROS_INFO("(MOTOR::START) STATUS is ready.");
  //Startup end
  
  MotorClass motor_start;
  std::cout<<"###############COMPLETE MOTOR INITILIZATION###############"<<std::endl;
  ros::spin();

  return(0);
}