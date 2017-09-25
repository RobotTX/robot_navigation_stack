#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <gobot_msg_srv/IsCharging.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <thread>

double cov_sum=0.0;
double cov_threshold = 0.3, rot_vel = 0.314, rot_time = 20;
bool goalActive = false,globalize_pose = false;
ros::Publisher vel_pub,goalCancel_pub;
std::string poseFile,homeFile;

bool rotateFindPose(double rot_v,double rot_t){
    double dt=0.0;
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z=rot_v;
    vel_pub.publish(vel);
    //rotate for time rot_t until find small covariance
    ros::Rate loop_rate(5);
    ros::Time last_time = ros::Time::now();
    while(globalize_pose && cov_sum>cov_threshold && dt<rot_t){
        dt=(ros::Time::now() - last_time).toSec();
        ROS_INFO("I am finding my pose in the map...covariance:%.3f",cov_sum);
        loop_rate.sleep();
        ros::spinOnce();
    }
    vel.linear.x=0.0;
    vel.angular.z=0.0;
    vel_pub.publish(vel);

    if(cov_sum<cov_threshold)
    {
        ROS_INFO("Find current robot pose in the map.");
        return true;
    }
    else
    {
        ROS_INFO("Unable to find current robot pose in the map.");
        return false;
    }
}

void checkGoalStatus(){
    if(goalActive)
    {
        actionlib_msgs::GoalID arg;
        goalCancel_pub.publish(arg);
        ROS_INFO("Cancelled active goal to proceed gobalo localization. Wait 3 sec to start global localization...");
        ros::Duration(3.0).sleep();
    }
}

bool GlobalLocalization(){
    checkGoalStatus();
    std_srvs::Empty arg;
    ros::service::call("/request_nomotion_update",arg);
    if(ros::service::call("/global_localization",arg))
        return rotateFindPose(rot_vel,rot_time*5.0);
    else
        ROS_ERROR("Failed to reset particles.");
}

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
        ros::Duration(2.0).sleep();

        initial_pose_publisher.publish(initialPose);

        ROS_INFO("(initial_pose_publisher) initialpose published");
    } else
        ROS_ERROR("(initial_pose_publisher) got a wrong position (robot probably got no home)");
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    cov_sum = 0.0;
    for (int i=0;i<msg->pose.covariance.size();i++){
        cov_sum += std::abs(msg->pose.covariance[i]);
    }
    ROS_INFO("Pose covariance sum:%f",cov_sum);

    //Write lastest amcl_pose to file
    std::ofstream ofs(poseFile, std::ofstream::out | std::ofstream::trunc);
    if(ofs.is_open()){
        ofs << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.orientation.x<<" "<< msg->pose.pose.orientation.y<<" "<< msg->pose.pose.orientation.z<<" "<< msg->pose.pose.orientation.w;
        ofs.close();
    } else
        ROS_ERROR("Could not open the file %s", poseFile.c_str());
}

void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if(msg->status_list.empty()) //no goal
        goalActive = false;
    else if (msg->status_list.back().status == 1) //has active goal
        //cancel goal
        goalActive = true;
    else
        goalActive =false;
}

bool checkInitPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    gobot_msg_srv::IsCharging arg;
    globalize_pose = true;
    ros::service::call("/sensors/isCharging",arg);
    //default pose is charging station 
    if(arg.response.isCharging){
        //if robot is charging, it is in CS station 
        return true;
    }
    else {
        //check amcl pose covariance by rotating on CS spot
        if(rotateFindPose(rot_vel,rot_time))
            //robot probably in CS station
            return true;
        else{
            //read last stop pose and initial robot pose
            double last_pos_x,last_pos_y,last_ang_x,last_ang_y,last_ang_z,last_ang_w;
            publishInitialpose(last_pos_x,last_pos_y,last_ang_x,last_ang_y,last_ang_z,last_ang_w);
            ros::Duration(2.5).sleep();
            if(rotateFindPose(rot_vel,rot_time))
                //robot probably in last stop pose
                return true;
            else
                return GlobalLocalization();
        }
    }

}

bool globalizePoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //check whether have active goal, and cancel it if have
    cov_sum = 100.0;
    globalize_pose = true;
    return GlobalLocalization();
}

bool stopGlobalizePoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    globalize_pose = false;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "initialpose_estimation");
    ros::NodeHandle nh;
    if(nh.hasParam("pose_file")){
        nh.getParam("pose_file", poseFile);
        ROS_INFO("(Command system) set pose file to %s", poseFile.c_str());
    } else
        ROS_ERROR("Could not find the param pose_file");

    if(nh.hasParam("home_file")){
        nh.getParam("home_file", homeFile);
        ROS_INFO("(Command system) set home file to %s", homeFile.c_str());
    } else
        ROS_ERROR("Could not find the param home_file");

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    goalCancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);
    ros::Subscriber goalStatus = nh.subscribe("/move_base/status",1,goalStatusCallback);
    ros::Subscriber initialPose = nh.subscribe("/amcl_pose",1,initialPoseCallback);
    ros::ServiceServer checkInitPose = nh.advertiseService("/gobot_recovery/check_init_pose",checkInitPoseCallback);
    ros::ServiceServer globalizePose = nh.advertiseService("/gobot_recovery/globalize_pose",globalizePoseCallback);
    ros::ServiceServer stopGlobalizePose = nh.advertiseService("/gobot_recovery/stop_globalize_pose",stopGlobalizePoseCallback);

    ros::spin();
    return 0;
}
