#include <ros/ros.h>
#include <std_srvs/Empty.h>




int main(int argc, char **argv) {
    ros::init(argc, argv, "startup");
    ros::NodeHandle nh;

    ros::Duration(5.0).sleep();

    std_srvs::Empty arg;
    
    while(!ros::service::call("/gobot_recovery/initialize_pose",arg)){
        ROS_ERROR("Failed to start robot");
        ros::service::call("/gobot_recovery/stop_globalize_pose",arg);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Start Robot.");

    ros::spin();
    return 0;
}