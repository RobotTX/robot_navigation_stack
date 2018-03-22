#include <ros/ros.h>
#include <signal.h>
#include <gobot_msg_srv/robot_command.h>

robot_class::RobotCommand robot_lib;

void mySigintHandler(int sig){   
  robot_lib.setMotorSpeed(0.0, 0.0);
  std::vector<std::string> color;
  color.push_back("white");
  robot_lib.setLed(0,color);
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hardware_test");
  ros::NodeHandle n;
  signal(SIGINT, mySigintHandler);

  robot_lib.initialize();
  robot_lib.stopCostmap();
  //@Programmable voice
  //ttsEnglish
  robot_lib.ttsEnglish("Hello, world.");
  
  //ttsChinese
  ros::Duration(3.0).sleep();
  robot_lib.ttsChinese("你好，世界。");
  
  

  //@Interact with base sensors such as color, sound and sensors data
  //setLed
  std::vector<std::string> color;
  color.push_back("red");
  ROS_INFO("Change LED to permanent red");
  robot_lib.setLed(0,color);
  ros::Duration(5.0).sleep();

  color.clear();
  color.push_back("green");
  color.push_back("white");
  ROS_INFO("Change LED to running green-white");
  robot_lib.setLed(1,color);
  ros::Duration(5.0).sleep();

  color.clear();
  color.push_back("blue");
  ROS_INFO("Change LED to permanent blue");
  robot_lib.setLed(0,color);
  ros::Duration(5.0).sleep();

  color.clear();
  color.push_back("cyan");
  color.push_back("magenta");
  color.push_back("yellow");
  ROS_INFO("Change LED to running cyan-magenta-yellow");
  robot_lib.setLed(1,color);

  //setSound
  ROS_INFO("Set sound num: 1, time_on: short");
  robot_lib.setSound(1,1);
  ros::Duration(5.0).sleep();

  ROS_INFO("Set sound num: 2, time_on: median");
  robot_lib.setSound(2,2);
  ros::Duration(5.0).sleep();

  ROS_INFO("Set sound num: 3, time_on: long");
  robot_lib.setSound(3,3);
  ros::Duration(5.0).sleep();

  robot_lib.setSound(4,1);
  ROS_INFO("Set sound num: 4, time_on: short");
  ros::Duration(5.0).sleep();

  //@Interact with base motors such as get/set motor speeds and encoders
  //setMotorSpeed
  ROS_INFO("Set motor speed LEFT:'F', 20; RIGHT:'B', 20.");
  robot_lib.setMotorSpeed('F', 20, 'B', 20);
  ros::Duration(5.0).sleep();
  ROS_INFO("Set motor speed LEFT:'F', 0; RIGHT:'B', 0.");
  robot_lib.setMotorSpeed('F', 0, 'B', 0);
  ros::Duration(5.0).sleep();

  //setMotorSpeed
  ROS_INFO("Set motor speed LINEAR:0.0; ANGULAR:0.3.");
  robot_lib.setMotorSpeed(0.0, 0.3);

  //setSpeedLimit
  double linear_limit = 0.2, angular_limit = 0.3;
  robot_lib.setSpeedLimit(linear_limit,angular_limit);

  while(ros::ok()){
    //getBatteryPercent
    int percentage = robot_lib.getBatteryPercent();
    int charging_current = robot_lib.getBatteryChargingCurrent();
    bool charging_status = robot_lib.getBatteryCharging();
    ROS_INFO("Battery percentage: %d, charging_current: %d, charging status: %d", percentage,charging_current,charging_status);

    //getGyro
    robot_class::Gyro gyro_data;
    robot_lib.getGyro(gyro_data);
    ROS_INFO("Gyro data: %d, %d, %d; Accelerometer data: %d, %d, %d", 
    gyro_data.gyrox,gyro_data.gyroy,gyro_data.gyroz,gyro_data.accelx,gyro_data.accely,gyro_data.accelz);
    
    //getEncoder
    int left_encoder, right_encoder;
    robot_lib.getEncoder(left_encoder,right_encoder);
    ROS_INFO("Encoder readings: left %d, right %d", left_encoder,right_encoder);

    //getMotorSpeed
    int left_vel, right_vel;
    robot_lib.getMotorSpeed(left_vel,right_vel);
    ROS_INFO("Wheel velocity readings: left %d, right %d", left_vel,right_vel);

    float load_weight = robot_lib.getWeight();
    ROS_INFO("Load weight: %f (kg)", load_weight);
    
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  //ros::spin();
  return 0;
}