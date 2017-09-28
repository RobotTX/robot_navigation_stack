#include <gobot_recovery/startup.h>

ros::Publisher vel_pub;

void mySigintHandler(int sig)
{
  ros::Time last_time = ros::Time::now();
  geometry_msgs::Twist vel;
  vel.linear.x=0.0;
  vel.angular.z=0.0;
  ROS_INFO("Wait 3 seconds to stop robot...");

  while((ros::Time::now() - last_time).toSec()<3.0){
    vel_pub.publish(vel);
  }

  ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "startup");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    ros::Duration(3.0).sleep();

    std_srvs::Empty arg;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    while(!ros::service::call("/gobot_recovery/initialize_pose",arg)){
        ROS_ERROR("Failed to start robot");
        ros::service::call("/gobot_recovery/stop_globalize_pose",arg);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Start Robot.");

    ros::spin();
    return 0;
}