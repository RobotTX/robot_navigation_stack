#include "gobot_function/scan_function.h"

int noFrontiersLeft(0),no_frontier_threshold(10),back_to_start_when_finished(0);
bool exploring(false), docking(false);
ros::Time new_goal_time;
std::vector<std::string> startPose;
std_srvs::Empty empty_srv;

robot_class::RobotMoveClass MoveRobot;


//****************************** CALLBACK ******************************

//****************************** SERVICE ******************************
/// Service to stop the exploration
bool stopExplorationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //check whether robot is docking because it is part of auto-scanning process if robot starts scanning from docking station
    if(docking){
        ros::service::call("/gobot_function/stop_docking", empty_srv);
    }
    if(exploring){
        MoveRobot.setStatus(21,"STOP_EXPLORING");
        stopExploration();
    }

    return true;
}


/// Called when the service startExploration is called, launches doExploration in a new thread
bool startExplorationSrv(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    if(!exploring){
        back_to_start_when_finished = req.data;
        //1-auto charging when complete.  2-go back to starting position when complete.  others-stop when complete
        if(back_to_start_when_finished==1){
            gobot_msg_srv::GetStringArray robot_pose;
            ros::service::call("/gobot_status/get_pose",robot_pose);
            if(robot_pose.response.data.size()==3){
                for(int i=0; i<robot_pose.response.data.size();i++)
                    startPose.push_back(robot_pose.response.data[i]);
                ROS_INFO("(SCAN_EXPLORE) Robot position : (%s, %s, %s)", startPose[0].c_str(),startPose[1].c_str(),startPose[2].c_str());
            }
        }
        //check whether robot is charging
        MoveRobot.moveFromCS();
        //reset new_goal_time to start exploration fast
        new_goal_time = ros::Time::now();
        exploring = true;
        docking = false;
        noFrontiersLeft = 0;
        
        MoveRobot.setStatus(22,"START_EXPLORING");

        std::thread(doExploration).detach();
    }   
    else{
        ROS_WARN("(SCAN_EXPLORE) We were already exploring");
    }
    return true;
}

//****************************** FUNCTIONS ******************************
/// To make the robot go back to its starting position
void backToStart(){
    ROS_INFO("(SCAN_EXPLORE) Back to start state: %d",back_to_start_when_finished);
    switch(back_to_start_when_finished){
        case 0:
            ROS_INFO("(SCAN_EXPLORE) Complete scanning and stop robot");
            break;

        case 1:
            if(startPose.size()==3){
                tf::Quaternion homeQua;
                homeQua.setRPY(0, 0, std::stod(startPose[2]));
                MoveRobot.setHome(startPose[0],startPose[1],
                                  std::to_string(homeQua.x()),std::to_string(homeQua.y()),std::to_string(homeQua.z()),std::to_string(homeQua.w()));

                ROS_INFO("(SCAN_EXPLORE) Complete exploration and send robot home:%s,%s",startPose[0].c_str(),startPose[1].c_str());

                if(ros::service::call("/gobot_function/start_docking", empty_srv))
                    docking = true;
            }
            break;

        default:
            ROS_ERROR("(SCAN_EXPLORE) back_to_start_when_finished value not defined : %d", back_to_start_when_finished);
            break;
    }
}

/// Call a service to get the trajectory for exploration from the hector_exploration_planner and send the goal to move_base
void doExploration(){
    ROS_INFO("(SCAN_EXPLORE) Starting exploration...");
    ros::Rate loop_rate(1);
    while(ros::ok() && exploring){
        /// We want to refresh the goal after a certain time or when we reached/aborted it
        if(ros::Time::now()>new_goal_time || 
            MoveRobot.getGoalState()==actionlib::SimpleClientGoalState::SUCCEEDED || 
            MoveRobot.getGoalState()==actionlib::SimpleClientGoalState::ABORTED){
            hector_nav_msgs::GetRobotTrajectory arg;
            /// Get an array of goals to follow
            if(ros::service::call("get_exploration_path", arg)){
                if(!arg.response.trajectory.poses.empty()){
                    noFrontiersLeft = 0;
                    move_base_msgs::MoveBaseGoal goal;
                    /// The array is a set of goal creating a trajectory but it's used in the hector_navigation stack
                    /// so we only extract the last goal and move_base will be in charge of going there
                    goal.target_pose = arg.response.trajectory.poses.back();
                    MoveRobot.moveTo(goal);
                    ROS_INFO("(SCAN_EXPLORE) Assign new goal to explore...");

                    MoveRobot.setStatus(25,"EXPLORING");
                    new_goal_time = ros::Time::now() + ros::Duration(60.0);
                } 
                else{
                    noFrontiersLeft++;
                    ROS_INFO("(SCAN_EXPLORE) No frontiers left, %d.",noFrontiersLeft);
                    /// After no_frontier_threshold attemps at finding a point to explore, we consider the scan finished
                    if(noFrontiersLeft >= no_frontier_threshold){
                        if(exploring){
                            MoveRobot.setStatus(21,"COMPLETE_EXPLORING");
                            stopExploration();
                            /// if we want to go back to the starting position
                            backToStart();
                        }
                        ROS_INFO("(SCAN_EXPLORE) Exploration finished");
                    }
                }
            } 
        } 
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/// To stop the exploration (either when the service is called or when we are done scanning)
void stopExploration(){
    ROS_INFO("(SCAN_EXPLORE) Stopped exploring");
    exploring = false;
    MoveRobot.cancelMove();
}

void initData(){
    ros::NodeHandle nh;
    nh.param<int>("no_frontier_threshold", no_frontier_threshold, 5);

	//Set lower auto speed for scan process
	MoveRobot.setAutoSpeedLimit(0.2, 0.5);

	//Set lower manual speed for scan process
	MoveRobot.setManualSpeedLimit(0.4, 0.8);
}

void mySigintHandler(int sig)
{   
    ros::shutdown();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "scan_function");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(SCAN_EXPLORE) Waiting for Robot setting hardware...");
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    ROS_INFO("(SCAN_EXPLORE) Robot setting hardware is ready.");
    //Startup end

    MoveRobot.moveClientInitialize();

    ROS_INFO("(SCAN_EXPLORE) running...");

    initData();

    /// Launch service's servers
    //0-don't go back to starting point; 1-go back to charging station; 2-go back to normal staring point
    ros::ServiceServer startExploration = nh.advertiseService("/gobot_scan/start_exploration", startExplorationSrv);
    ros::ServiceServer stopExploration = nh.advertiseService("/gobot_scan/stop_exploration", stopExplorationSrv);

    MoveRobot.setStatus(20,"EXPLORATION");

    ros::spin();

    return 0;
}