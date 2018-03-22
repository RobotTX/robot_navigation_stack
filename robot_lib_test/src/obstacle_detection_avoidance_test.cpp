#include <ros/ros.h>
#include <gobot_msg_srv/robot_command.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection_avoidance_test");
  ros::NodeHandle n;

  robot_class::RobotCommand robot_lib;
  robot_lib.initialize();
  
  //@Obstacle Detection & Avoidance

  //get laser data
  sensor_msgs::LaserScan laser_data;
  robot_lib.getLaserData(laser_data);
  ROS_INFO("No. of laser beams: %zu", laser_data.ranges.size());

  //get plan path for given start pose and goal pose
  geometry_msgs::PoseStamped start, goal;
  start.header.stamp = ros::Time::now();
  start.header.frame_id = "map";
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;

  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";
  goal.pose.position.x = 10.0;
  goal.pose.position.y = 10.0;
  goal.pose.orientation.w = 1.0;
  std::vector<geometry_msgs::PoseStamped> make_plan;
  if (robot_lib.makePlanPath(start,goal,make_plan)){
    ROS_INFO("No. of poses along planned path: %zu from (%f, %f) to (%f, %f)", make_plan.size(),start.pose.position.x,start.pose.position.y,
    goal.pose.position.x,goal.pose.position.y);
  }
  else{
    ROS_WARN("Can not find a path for given start pose and goal pose.");
  }

  //get planned path from current pose to given goal pose
  geometry_msgs::PoseStamped goal2;
  goal2.header.stamp = ros::Time::now();
  goal2.header.frame_id = "map";
  goal2.pose.position.x = 0.0;
  goal2.pose.position.y = 0.0;
  goal2.pose.orientation.w = 1.0;
  std::vector<geometry_msgs::PoseStamped> make_plan2;
  if (robot_lib.makePlanPath(goal2,make_plan2)){
    ROS_INFO("No. of poses along planned path: %zu from current pose to (%f, %f)", make_plan2.size(),goal2.pose.position.x,goal2.pose.position.y);
  }
  else{
    ROS_WARN("Can not find a path for current pose and given goal pose.");
  }

  //clear costmap 
  robot_lib.clearCostMap();

  while(ros::ok()){
    //get planned path to goal pose if have
    std::vector<geometry_msgs::PoseStamped> planned_path;
    robot_lib.getPlanPath(planned_path);
    ROS_INFO("No. of poses along planned path: %zu", planned_path.size());
    
    //get four sonars data
    std::vector<int> sonar_data;
    robot_lib.getSonarData(sonar_data);
    ROS_INFO("Sonar Data 1: %d, 2: %d, 3: %d, 4: %d",sonar_data[0],sonar_data[1],sonar_data[2],sonar_data[3]);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  //ros::spin();
  return 0;
}