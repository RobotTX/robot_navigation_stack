#include <hector_exploration_node/move_base_controller.hpp>

int count(0);
int noFrontiersLeft(0);
bool exploring(false);

double new_goal_frequency(0.1);
int no_frontier_threshold(10);
int back_to_start_when_finished(0);
geometry_msgs::Pose startingPose;
std::string map_frame;
std::string base_frame;
/// Simple Action client
std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;

ros::Publisher exploration_pub;

/// To get the starting position
bool getRobotPos(void){
    try {
        tf::TransformListener listener;
        /// Wait for the transform between the map_frame and the base_frame
        listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(2.0));

        tf::StampedTransform transform;
        /// Get the transformation from the map_frame to the base_frame into transform
        listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

        /// Construct the starting pose
        startingPose.orientation.x = transform.getRotation().getX();
        startingPose.orientation.y = transform.getRotation().getY();
        startingPose.orientation.z = transform.getRotation().getZ();
        startingPose.orientation.w = transform.getRotation().getW();
        startingPose.position.x = transform.getOrigin().getX();
        startingPose.position.y = transform.getOrigin().getY();
        startingPose.position.z = transform.getOrigin().getZ();

        ROS_INFO("(move_base_controller) Robot position : (%f, %f, %f) - Robot orientation : (%f, %f, %f, %f)", 
            startingPose.position.x, startingPose.position.y,
            startingPose.position.z, startingPose.orientation.x,
            startingPose.orientation.y, startingPose.orientation.z,
            startingPose.orientation.w);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("(move_base_controller) Got a transform problem : %s", ex.what());
        return false;
    }

    return true;
}

/// To make the robot go back to its starting position
void backToStart(){
    ROS_INFO("(move_base_controller) Back to start launched...");
    switch(back_to_start_when_finished){
        case 1:
        {
            std_srvs::Empty arg;
            if(ros::service::call("/gobot_function/startDocking", arg))
                ROS_INFO("(move_base_controller) Trying to go back to a charging station after mapping");
            else 
                ROS_ERROR("(move_base_controller) Couldn't call service /gobot_function/startDocking");
        }
        break;
        case 2:
        {
            /// We go back to a normal point (not a charging station)
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = startingPose;
            
            if(ac->isServerConnected()){
                ac->sendGoalAndWait(goal, ros::Duration(0), ros::Duration(0));
                ROS_INFO("(move_base_controller) Start position goal achieved");
            } else 
                ROS_ERROR("(move_base_controller) No action server to go back to start");
        }
        break;
        default:
            ROS_ERROR("(move_base_controller) back_to_start_when_finished value not defined : %d", back_to_start_when_finished);
        break;
    }
}

/// Call a service to get the trajectory for exploration from the hector_exploration_planner and send the goal to move_base
void doExploration(void){
    ROS_INFO("(move_base_controller) Starting exploration...");

    exploring = true;

    ros::Rate loop_rate(5);
    while(ros::ok() && exploring){
        /// We want to refresh the goal after a certain time or when we reached it
        if(count >= (5 / new_goal_frequency) || ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            hector_nav_msgs::GetRobotTrajectory arg;
            /// Get an array of goals to follow
            if(ros::service::call("get_exploration_path", arg)){
                std::vector<geometry_msgs::PoseStamped> poses = arg.response.trajectory.poses;
                if(!poses.empty()){
                    ROS_INFO("(move_base_controller) Moving to frontier...");
                    noFrontiersLeft = 0;
                    move_base_msgs::MoveBaseGoal goal;
                    /// The array is a set of goal creating a trajectory but it's used in the hector_navigation stack
                    /// so we only extract the last goal and move_base will be in charge of going there
                    goal.target_pose = poses.back();

                    if(ac->isServerConnected() && exploring)
                        /// Send the goal to move_base
                        ac->sendGoal(goal);
                    else 
                        ROS_INFO("(move_base_controller) No action server or we stopped exploring already");
                } else {
                    ROS_INFO("(move_base_controller) No frontiers left.");
                    noFrontiersLeft++;
                    /// After no_frontier_threshold attemps at finding a point to explore, we consider the scan finished
                    if(noFrontiersLeft >= no_frontier_threshold){
                        std_srvs::Empty arg2;
                        stopExploration();

                        /// if we want to go back to the starting position
                        if(back_to_start_when_finished)
                            backToStart();

                        ROS_INFO("(move_base_controller) Calling service finished_auto_scan");
                        /// Service to tell command_system in the gobot_software package that we finished scanning
                        ros::service::call("finished_auto_scan", arg2);
                    }
                }

                count = 0;
            } else
                ROS_WARN("(move_base_controller) get_exploration_path service call failed");
        } else
            count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    std_msgs::Int8 result;
    result.data=1;
    exploration_pub.publish(result);
    ROS_WARN("(move_base_controller) Exploration finished");
}

/// Called when the service startExploration is called, launches doExploration in a new thread
bool startExplorationSrv(hector_exploration_node::Exploration::Request &req, hector_exploration_node::Exploration::Response &res){
    if(!exploring){
        back_to_start_when_finished = req.backToStartWhenFinished;
        /// if we want to go back to our starting position at the end of the scan, we need to save the starting position
        if(back_to_start_when_finished != 0){
            if(getRobotPos()){
                std::thread(doExploration).detach();
                return true;
            } else 
                return false;
        } else {
            std::thread(doExploration).detach();
            return true;
        }
    } else
        ROS_WARN("(move_base_controller) We were already exploring");
    return true;

}

/// Service to stop the exploration
bool stopExplorationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return stopExploration();
}

/// To stop the exploration (either when the service is called or when we are done scanning)
bool stopExploration(void){
    ROS_INFO("(move_base_controller) stopExploration called %d", exploring);
    if(exploring){
        ROS_INFO("(move_base_controller) Stopped exploring");
        exploring = false;
        ac->cancelAllGoals();
    } else
        ROS_WARN("(move_base_controller) We were not exploring");
    
    return true;
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "move_base_controller");
    ros::NodeHandle nh;

    ROS_INFO("(move_base_controller) running...");

    /// Get the data from the parameters server
    nh.param<std::string>("map_frame", map_frame, "map");
    ROS_INFO("(move_base_controller) map_frame : %s", map_frame.c_str());

    nh.param<std::string>("base_frame", base_frame, "base_link");
    ROS_INFO("(move_base_controller) base_frame : %s", base_frame.c_str());

    nh.param<double>("new_goal_frequency", new_goal_frequency, 0.1);
    ROS_INFO("(move_base_controller) new_goal_frequency : %f", new_goal_frequency);

    nh.param<int>("no_frontier_threshold", no_frontier_threshold, 5);
    ROS_INFO("(move_base_controller) no_frontier_threshold : %d", no_frontier_threshold);

    //Publish when exploration is complete
    exploration_pub = nh.advertise<std_msgs::Int8>("exploration_result",10);

    /// Create an actionlibClient to be able to send and monitor goals
    ac = std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));

    /// Launch service's servers
    //0-don't go back to starting point; 1-go back to charging station; 2-go back to normal staring point
    ros::ServiceServer startExploration = nh.advertiseService("/gobot_scan/startExploration", startExplorationSrv);
    ros::ServiceServer stopExploration = nh.advertiseService("/gobot_scan/stopExploration", stopExplorationSrv);

    ros::spin();

    return 0;
}