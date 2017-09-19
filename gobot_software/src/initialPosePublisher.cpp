#include <iostream>
#include <fstream>
#include <stdio.h>
#include "ros/ros.h"
#include <time.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gobot_base/IsCharging.h"

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

int main(int argc, char* argv[] ){

    try { 

        ros::init(argc, argv, "initial_pose_publisher");
        ros::NodeHandle n; 

        double position_x(0.0), position_y(0.0), angle_x(0.0), angle_y(0.0), angle_z(0.0), angle_w(0.0);

        std::string robotPos;

        // from this file we retrieve an int that tells us if the robot should start at the charging station
        // or at the last known position after scanning
        if(n.hasParam("robot_position_file")){
            std::string robotPositionFile;
            n.getParam("robot_position_file", robotPositionFile);

            std::fstream file(robotPositionFile, std::ios::in);
            file >> robotPos;
            file.close();

            file.open(robotPositionFile, std::fstream::out | std::fstream::trunc);
            file << "1";
            file.close();


            ROS_INFO("(initial_pose_publisher) RobotPos : %s", robotPos.c_str());
            /// If we were scanning, we want the robot to start were it stopped and not at the charging station
            if(std::stoi(robotPos) == 0){
                if(n.hasParam("last_known_position_file")){
                    std::string fileName;
                    n.getParam("last_known_position_file", fileName);
                    std::ifstream file(fileName, std::ios::in);
                    if(file){
                        file >> position_x >> position_y >> angle_x >> angle_y >> angle_z >> angle_w;
                        ROS_INFO("(initial_pose_publisher) values I got from lastKnownPosition.txt [%f, %f] [%f, %f, %f, %f]", position_x, position_y, angle_x, angle_y, angle_z, angle_w);
                        file.close();
                        publishInitialpose(position_x, position_y, angle_x, angle_y, angle_z, angle_w);
                    }
                } else 
                    ROS_ERROR("(initial_pose_publisher) could not find the parameter <last_known_position_file>");
                    
            } else if(std::stoi(robotPos) == 1){
                /// If we weren't scanning, we might have move so we if we are charging, we assume we are at our CS
                gobot_base::IsCharging arg;
                if(ros::service::call("isCharging", arg)){
                    if(arg.response.isCharging){
                        if(n.hasParam("home_file")){
                            std::string fileName;
                            n.getParam("home_file", fileName);
                            std::ifstream file(fileName, std::ios::in);
                            if(file){
                                file >> position_x >> position_y >> angle_x >> angle_y >> angle_z >> angle_w;
                                ROS_INFO("(initial_pose_publisher) values I got from home.txt [%f, %f] [%f, %f, %f, %f]", position_x, position_y, angle_x, angle_y, angle_z, angle_w);
                                file.close();
                                publishInitialpose(position_x, position_y, angle_x, angle_y, angle_z, angle_w);
                            } 
                        } else 
                            ROS_ERROR("(initial_pose_publisher) could not find the parameter <home_file>");
                    } else 
                        ROS_WARN("(initial_pose_publisher) The robot is not charging so we do not set its initialPose");
                } else
                    ROS_ERROR("(initial_pose_publisher) could not call service isCharging");
            }
        } else 
            ROS_ERROR("(initial_pose_publisher) could not find the parameter </initialPosePublisher/robot_position_file>");


        ros::spinOnce();

    } catch (std::exception& e) {
        ROS_ERROR("(initial_pose_publisher) Exception: %s", e.what());
    }

    return 0;
}
