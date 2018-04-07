#include <ros/ros.h>
#include <gobot_msg_srv/robot_command.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_route_test");
  ros::NodeHandle n;

  robot_class::RobotCommand robot_lib;
  robot_lib.initialize();

  //@Target Points || Routes
  /*//getPointCost
  double point_x = 0.0, point_y = 0.0;
  int point_cost = robot_lib.getPointCost(point_x,point_y);
  ROS_INFO("The cost value of point (%.2f,%.2f) is %d", point_x, point_y, point_cost);
  */
  
  /*//setTargetPoint
  robot_class::Pose point1;
  point1.x = 2.0;
  point1.y = 2.0;
  point1.theta = 0.0;
  bool sentPoint = robot_lib.setTargetPoint(point1);
  if (sentPoint)
    ROS_INFO("Successfully send target point to robot.");
  else
    ROS_INFO("Failed to send target point to robot.");
  */

  /*//setTargetPoint
  geometry_msgs::PoseStamped point2;
  point2.header.stamp = ros::Time::now();
  point2.header.frame_id = "map";
  point2.pose.position.x = 2.0;
  point2.pose.position.y = 2.0;
  point2.pose.orientation.w = 1.0;
  bool sentPoint = robot_lib.setTargetPoint(point2);
  if (sentPoint)
    ROS_INFO("Successfully send target point to robot.");
  else
    ROS_INFO("Failed to send target point to robot.");
  */

  /*//setTargetPath
  robot_class::Path path;
  std::vector<robot_class::Path> test_path;
  path.x = 1.0;
  path.y = 1.0;
  path.waiting = 5;
  path.theta = 90;
  test_path.push_back(path);
  path.name = "P2";
  path.x = 2.0;
  path.y = 2.0;
  path.waiting = 5;
  path.theta = 0;
  test_path.push_back(path);
  robot_lib.setTargetPath(test_path,"test_path");
  */


  while(ros::ok()){
    //getCurrentGoal
    geometry_msgs::PoseStamped current_goal;
    robot_lib.getCurrentGoal(current_goal);
    ROS_INFO("Current goal: x-%f, y-%f, quaternion-(0.0,0.0,%f,%f)", current_goal.pose.position.x,current_goal.pose.position.y,
    current_goal.pose.orientation.z,current_goal.pose.orientation.w);

    //getGoalStatus
    int goal_status = robot_lib.getGoalStatus();
    ROS_INFO("Current goal status: %d",goal_status);

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }


  //@Control robot motion
  //robot_lib.playTargetPath();
  //robot_lib.pauseRobot();
  //robot_lib.stopRobot();
  //robot_lib.startDock();
  //robot_lib.stopDock();
  //robot_lib.shutDown();

  //ros::spin();
  return 0;
}