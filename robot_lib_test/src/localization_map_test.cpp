#include <ros/ros.h>
#include <gobot_msg_srv/robot_command.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization_map_test");
  ros::NodeHandle n;

  robot_class::RobotCommand robot_lib;
  robot_lib.initialize();

  //@Map
  /*//saveMapTo
  robot_lib.saveMapTo();
  ros::Duration(3.0).sleep();
  std::string map_path = "~/testmap";
  robot_lib.saveMapTo(map_path);
  */

  /*//getMapMetadata
  robot_class::Map map_data;
  robot_lib.getMapMetadata(map_data);
  ROS_INFO("Map resolution: %f",map_data.resolution);
  ROS_INFO("Map width and height in pixel (%d, %d)",map_data.pixel_width, map_data.pixel_height);
  ROS_INFO("Map width and height in meter (%f, %f)",map_data.meter_width, map_data.meter_height);
  ROS_INFO("Map origin(x:%f, y:%f, theta:%f)",map_data.origin.x,map_data.origin.y, map_data.origin.theta);
  */
  
  while(ros::ok()){
    //@Localization
    //getCurrentPose
    robot_class::Pose current_pose1;
    robot_lib.getCurrentPose(current_pose1);
    ROS_INFO("Current robot pose:(x:%f, y:%f, theta:%f)", current_pose1.x,current_pose1.y,current_pose1.theta);
    
    
    //getCurrentPose
    geometry_msgs::PoseStamped current_pose2;
    robot_lib.getCurrentPose(current_pose2);
    ROS_INFO("Current robot pose:(x:%f, y:%f, quaternion:0.0,0.0,%f,%f)", current_pose2.pose.position.x,current_pose2.pose.position.y,
    current_pose2.pose.orientation.z,current_pose2.pose.orientation.w);
    
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  //ros::spin();
  return 0;
}