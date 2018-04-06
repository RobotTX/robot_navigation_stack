#include <gobot_base/wheels_node.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "wheel_node");

  std::cout<<"I AM OUT"<<std::endl;
  MotorClass motor_start;

  std::cout<<"I AM OUT2"<<std::endl;
  ros::spin();

  return(0);
}