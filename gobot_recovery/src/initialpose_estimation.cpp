#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <gobot_msg_srv/IsCharging.h>


double cov_sum=0.0;
ros::Publisher velocity_pub;

void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w){

    if(position_x != 0 || position_x != 0 || position_x != 0 || position_x != 0 || position_x != 0 || position_x != 0){
        ros::NodeHandle n; 

        geometry_msgs::PoseWithCovarianceStamped initialPose;
        initialPose.header.frame_id = "map";
        initialPose.header.stamp = ros::Time::now();
        initialPose.pose.pose.position.x = position_x;
        initialPose.pose.pose.position.y = position_y;
        initialPose.pose.pose.orientation.x = angle_x;
        initialPose.pose.pose.orientation.y = angle_y;
        initialPose.pose.pose.orientation.z = angle_z;
        initialPose.pose.pose.orientation.w = angle_w;

        ros::Publisher initial_pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        
        // we wait for amcl to launch
        sleep(3);

        initial_pose_publisher.publish(initialPose);

        ROS_INFO("(initial_pose_publisher) initialpose published");
    } else
        ROS_ERROR("(initial_pose_publisher) got a wrong position (robot probably got no home)");
}

bool checkInitPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    gobot_msg_srv::IsCharging arg;
    ros::service::call("isCharging",arg);
    
    if(arg.response.IsCharging){
        //if charging, use CS pose as initial pose
    }
    else {
        //check amcl pose covariance by rotating on CS spot
        rotate(vel,)
        if(cov_cum<0.25)
        {   
            //robot probably in CS station
            publishInitialpose(pos_x,_pos_y,ang_x,ang_y,ang_z,ang_w)
        }
        else
        {

        }
    }

}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    cov_sum = 0.0;
    for (int i=0;i<msg->pose.covariance.size();i++){
        cov_sum += std::abs(msg->pose.covariance[i]);
    }
    ROS_INFO("Pose covariance sum:%f",cov_sum);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "initialpose_estimation");
    ros::NodeHandle nh;

    ros::Subscriber initialPose = nh.subscribe("/amcl_pose",1,initialPoseCallback);
    ros::ServiceServer allowTebSrv = nh.advertiseService("/gobot_recovery/check_init_pose",checkInitPoseCallback);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::spin();
    return 0;
}