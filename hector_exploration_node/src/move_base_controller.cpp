#include <ros/ros.h>
#include <iostream>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <thread>

int count(0);
bool exploring(false);

/// Simple Action client
std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;

void doExploration(){
    ROS_INFO("(move_base_controller) Starting exploration...");

    exploring = true;

    ros::Rate loop_rate(10);
    while(ros::ok() && exploring){

        if(count >= 20 || ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            hector_nav_msgs::GetRobotTrajectory arg;
            if(ros::service::call("get_exploration_path", arg)){
                std::vector<geometry_msgs::PoseStamped> poses = arg.response.trajectory.poses;

                if(!poses.empty()){
                    ROS_INFO("(move_base_controller) Moving to frontier...");
                    /// Move to goal
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose = poses.back();

                    if(ac->isServerConnected())
                        ac->sendGoal(goal);
                    else 
                        ROS_INFO("(move_base_controller) no action server");
                } else {
                    ROS_INFO("(move_base_controller) No frontiers left.");
                    std_srvs::Empty arg2;
                    ros::service::call("finished_auto_scan", arg2);
                }

                count = 0;
            } else
                ROS_WARN("(move_base_controller) get_exploration_path service call failed");
        } else
            count++;

        ros::spinOnce();
        loop_rate.sleep();
    }
}


bool startExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(!exploring){
        std::thread(doExploration).detach();
        return true;
    } else {
        ROS_ERROR("(move_base_controller) We are already exploring");
        return false;
    }
}

bool stopExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(exploring){
        exploring = false;
        ac->cancelAllGoals();
        return true;
    } else {
        ROS_ERROR("(move_base_controller) We are not exploring");
        return false;
    }
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "move_base_controller");
    ros::NodeHandle nh("move_base_controller");

    ROS_INFO("(move_base_controller) running...");

    ac = std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));

    ros::ServiceServer startExplorationSrv = nh.advertiseService("startExploration", startExploration);
    ros::ServiceServer stopExplorationSrv = nh.advertiseService("stopExploration", stopExploration);

    ros::spin();

    return 0;
}