#include <gobot_software/scan_controller.hpp>

int noFrontiersLeft(0),no_frontier_threshold(10),back_to_start_when_finished(0);
bool exploring(false), docking(false);
ros::Time start_goal_time;
geometry_msgs::Pose startingPose;
std::string map_frame;
std::string base_frame;
std_srvs::Empty empty_srv;
/// Simple Action client
MoveBaseClient* ac;

robot_class::SetRobot SetRobot;

/// To get the starting position
bool getRobotPos(){
    gobot_msg_srv::GetStringArray robot_pose;
    ros::service::call("/gobot_status/get_pose",robot_pose);
    if(robot_pose.response.data.size()==3){
        startingPose.position.x = std::stod(robot_pose.response.data[0]);
        startingPose.position.y = std::stod(robot_pose.response.data[1]);
        startingPose.orientation = tf::createQuaternionMsgFromYaw(std::stod(robot_pose.response.data[2]));
        ROS_INFO("(SCAN_EXPLORE) Robot position : (%f, %f, %f) - Robot orientation : (%f, %f, %f, %f)", 
        startingPose.position.x, startingPose.position.y,startingPose.position.z, 
        startingPose.orientation.x,startingPose.orientation.y, startingPose.orientation.z,startingPose.orientation.w);
    }
    return true;
}

/// To make the robot go back to its starting position
void backToStart(){
    ROS_INFO("(SCAN_EXPLORE) Back to start state: %d",back_to_start_when_finished);
    move_base_msgs::MoveBaseGoal goal;
    switch(back_to_start_when_finished){
        case 0:
            ROS_INFO("(SCAN_EXPLORE) Complete scanning and stop robot");
            break;
        case 1:
            SetRobot.setHome(std::to_string(startingPose.position.x),std::to_string(startingPose.position.y),std::to_string(startingPose.orientation.x),
            std::to_string(startingPose.orientation.y),std::to_string(startingPose.orientation.z),std::to_string(startingPose.orientation.w));

            ROS_INFO("(SCAN_EXPLORE) Complete exploration and send robot home:%.2f,%.2f",startingPose.position.x,startingPose.position.y);

            if(ros::service::call("/gobot_function/startDocking", empty_srv)){
                docking = true;
            }
            break;
        default:
            ROS_ERROR("(SCAN_EXPLORE) back_to_start_when_finished value not defined : %d", back_to_start_when_finished);
            break;
    }
}

/// Call a service to get the trajectory for exploration from the hector_exploration_planner and send the goal to move_base
void doExploration(void){
    ROS_INFO("(SCAN_EXPLORE) Starting exploration...");
    ros::Rate loop_rate(1);
    while(ros::ok() && exploring){
        /// We want to refresh the goal after a certain time or when we reached/aborted it
        if(ros::Time::now()>start_goal_time || ac->getState()==actionlib::SimpleClientGoalState::SUCCEEDED || ac->getState()==actionlib::SimpleClientGoalState::ABORTED){
            hector_nav_msgs::GetRobotTrajectory arg;
            /// Get an array of goals to follow
            if(ros::service::call("get_exploration_path", arg)){
                std::vector<geometry_msgs::PoseStamped> poses = arg.response.trajectory.poses;
                if(!poses.empty()){
                    noFrontiersLeft = 0;
                    move_base_msgs::MoveBaseGoal goal;
                    /// The array is a set of goal creating a trajectory but it's used in the hector_navigation stack
                    /// so we only extract the last goal and move_base will be in charge of going there
                    goal.target_pose = poses.back();
                    if(ac->isServerConnected() && exploring){
                        ROS_INFO("(SCAN_EXPLORE) Assign new goal to explore...");
                        /// Send the goal to move_base
                        ac->sendGoal(goal);
                        SetRobot.setStatus(25,"EXPLORING");
                        start_goal_time = ros::Time::now() + ros::Duration(30.0);
                    }
                } 
                else{
                    noFrontiersLeft++;
                    ROS_INFO("(SCAN_EXPLORE) No frontiers left, %d.",noFrontiersLeft);
                    /// After no_frontier_threshold attemps at finding a point to explore, we consider the scan finished
                    if(noFrontiersLeft >= no_frontier_threshold){
                        if(exploring){
                            SetRobot.setStatus(21,"COMPLETE_EXPLORING");
                            stopExploration();
                            /// if we want to go back to the starting position
                            backToStart();
                        }

                        ROS_INFO("(SCAN_EXPLORE) Exploration finished");
                    }
                }
            } 
            else{
                ROS_WARN("(SCAN_EXPLORE) get_exploration_path service call failed");
            }
        } 

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/// Called when the service startExploration is called, launches doExploration in a new thread
bool startExplorationSrv(hector_exploration_node::Exploration::Request &req, hector_exploration_node::Exploration::Response &res){
    if(!exploring){
        back_to_start_when_finished = req.backToStartWhenFinished;
        //1-auto charging when complete.  2-go back to starting position when complete.  others-stop when complete
        if(back_to_start_when_finished==1){
            getRobotPos();
        }
        gobot_msg_srv::IsCharging isCharging;
        if(ros::service::call("/gobot_status/charging_status", isCharging) && isCharging.response.isCharging){
            ROS_WARN("(SCAN_EXPLORE) Go straight because of charging.");
            SetRobot.setNavSpeed('F', 15, 'F', 15);
		    ros::Duration(2.5).sleep();
            SetRobot.setNavSpeed('F', 0, 'F', 0);
            SetRobot.setDock(0);
        }
        //reset start_goal_time to start exploration fast
        start_goal_time = ros::Time::now();
        exploring = true;
        docking = false;
        std::thread(doExploration).detach();
        return true;

    } 
    else{
        ROS_WARN("(SCAN_EXPLORE) We were already exploring");
    }
    return true;

}

/// Service to stop the exploration
bool stopExplorationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //check whether robot is docking because it is part of auto-scanning process if robot starts scanning from docking station
    if(docking){
        ros::service::call("/gobot_function/stopDocking", empty_srv);
    }
    if(exploring){
        SetRobot.setStatus(21,"STOP_EXPLORING");
        stopExploration();
    }

    return true;
}

/// To stop the exploration (either when the service is called or when we are done scanning)
bool stopExploration(void){
    //~ROS_INFO("(SCAN_EXPLORE) stopExploration called %d", exploring);
    if(exploring){
        ROS_INFO("(SCAN_EXPLORE) Stopped exploring");
        exploring = false;
        ac->cancelAllGoals();
    } 
    else{
        ROS_WARN("(SCAN_EXPLORE) We were not exploring");
    }
    
    return true;
}

void initData(){
    ros::NodeHandle nh;
    /// Get the data from the parameters server
    nh.param<std::string>("map_frame", map_frame, "map");
    nh.param<std::string>("base_frame", base_frame, "base_link");
    nh.param<int>("no_frontier_threshold", no_frontier_threshold, 5);

    //Set lower speed for scan process
    dynamic_reconfigure::Reconfigure config;
    dynamic_reconfigure::DoubleParameter linear_param,angular_param;
    linear_param.name = "max_vel_x";
    linear_param.value = 0.2;
    angular_param.name = "max_vel_theta";
    angular_param.value = 0.5;
    config.request.config.doubles.push_back(linear_param);
    config.request.config.doubles.push_back(angular_param);
    ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config);


    //Set lower manual speed for scan process
    gobot_msg_srv::SetFloatArray joy_speed;
    joy_speed.request.data.push_back(0.2);
    joy_speed.request.data.push_back(0.5);
    ros::service::call("/gobot_base/set_joy_speed",joy_speed);
}

void mySigintHandler(int sig)
{   
	delete ac;
    ros::shutdown();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "scan_controller");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    
    SetRobot.initialize();
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(SCAN_EXPLORE) Waiting for Robot setting hardware...");
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    ROS_INFO("(SCAN_EXPLORE) Robot setting hardware is ready.");
    //Startup end

    ROS_INFO("(SCAN_EXPLORE) running...");

    initData();

    /// Create an actionlibClient to be able to send and monitor goals
    ac = new MoveBaseClient("move_base", true);
    ac->waitForServer(ros::Duration(60.0));

    /// Launch service's servers
    //0-don't go back to starting point; 1-go back to charging station; 2-go back to normal staring point
    ros::ServiceServer startExploration = nh.advertiseService("/gobot_scan/startExploration", startExplorationSrv);
    ros::ServiceServer stopExploration = nh.advertiseService("/gobot_scan/stopExploration", stopExplorationSrv);

    SetRobot.setStatus(20,"EXPLORATION");

    ros::spin();

    return 0;
}