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
std_srvs::Empty empty_srv;
/// Simple Action client
std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac;

ros::Publisher exploration_pub;
gobot_msg_srv::SetGobotStatus set_gobot_status;
gobot_msg_srv::GetGobotStatus get_gobot_status;
ros::ServiceClient getGobotStatusSrv;
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

        ROS_INFO("(Exploration) Robot position : (%f, %f, %f) - Robot orientation : (%f, %f, %f, %f)", 
        startingPose.position.x, startingPose.position.y,
        startingPose.position.z, startingPose.orientation.x,
        startingPose.orientation.y, startingPose.orientation.z,
        startingPose.orientation.w);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("(Exploration) Got a transform problem : %s", ex.what());
        return false;
    }

    return true;
}

/// To make the robot go back to its starting position
void backToStart(){
    ROS_INFO("(Exploration) Back to start launched...");
    move_base_msgs::MoveBaseGoal goal;
    gobot_msg_srv::SetString set_home;
    switch(back_to_start_when_finished){
        case 1:
            set_home.request.data.push_back(std::to_string(startingPose.position.x));
            set_home.request.data.push_back(std::to_string(startingPose.position.y));
            set_home.request.data.push_back(std::to_string(startingPose.orientation.x));
            set_home.request.data.push_back(std::to_string(startingPose.orientation.y));
            set_home.request.data.push_back(std::to_string(startingPose.orientation.z));
            set_home.request.data.push_back(std::to_string(startingPose.orientation.w));
            ros::service::call("/gobot_status/set_home",set_home);
            ROS_INFO("(Exploration) Complete exploration and Set robot home:%.2f,%.2f",startingPose.position.x,startingPose.position.y);

            if(ros::service::call("/gobot_function/startDocking", empty_srv))
                ROS_INFO("(Exploration) Complete exploration and Sending robot home");
            break;
        case 2:
            /// We go back to a normal point (not a charging station)
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = startingPose;
            
            if(ac->isServerConnected()){
                ac->sendGoalAndWait(goal, ros::Duration(0), ros::Duration(0));
                ROS_INFO("(Exploration) Start position goal achieved");
            } 
            else 
                ROS_ERROR("(Exploration) No action server to go back to start");
            break;
        default:
            ROS_ERROR("(Exploration) back_to_start_when_finished value not defined : %d", back_to_start_when_finished);
            break;
    }
}

/// Call a service to get the trajectory for exploration from the hector_exploration_planner and send the goal to move_base
void doExploration(void){
    ROS_INFO("(Exploration) Starting exploration...");

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
                    ROS_INFO("(Exploration) Moving to frontier...");
                    noFrontiersLeft = 0;
                    move_base_msgs::MoveBaseGoal goal;
                    /// The array is a set of goal creating a trajectory but it's used in the hector_navigation stack
                    /// so we only extract the last goal and move_base will be in charge of going there
                    goal.target_pose = poses.back();

                    if(ac->isServerConnected() && exploring)
                        /// Send the goal to move_base
                        ac->sendGoal(goal);
                    else 
                        ROS_INFO("(Exploration) No action server or we stopped exploring already");
                } 
                else {
                    ROS_INFO("(Exploration) No frontiers left.");
                    noFrontiersLeft++;
                    /// After no_frontier_threshold attemps at finding a point to explore, we consider the scan finished
                    if(noFrontiersLeft >= no_frontier_threshold){
                        if(exploring){
                            std_msgs::Int8 result;
                            result.data=1;
                            exploration_pub.publish(result);

                            set_gobot_status.request.status = 21;
                            set_gobot_status.request.text = "COMPLETE_EXPLORING";
                            ros::service::call("/gobot_status/set_gobot_status",set_gobot_status);
                        }
                        stopExploration();

                        /// if we want to go back to the starting position
                        if(back_to_start_when_finished)
                            backToStart();

                        ROS_INFO("(Exploration) Exploration finished");
                    }
                }

                count = 0;
            } else
                ROS_WARN("(Exploration) get_exploration_path service call failed");
        } else
            count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/// Called when the service startExploration is called, launches doExploration in a new thread
bool startExplorationSrv(hector_exploration_node::Exploration::Request &req, hector_exploration_node::Exploration::Response &res){
    if(!exploring){
        std_msgs::Int8 result;
        result.data=-1;
        exploration_pub.publish(result);

        set_gobot_status.request.status = 25;
        set_gobot_status.request.text = "EXPLORING";
        ros::service::call("/gobot_status/set_gobot_status",set_gobot_status);
        
        back_to_start_when_finished = req.backToStartWhenFinished;

        if(back_to_start_when_finished==1){
            getRobotPos();
        }
        gobot_msg_srv::IsCharging isCharging;
        if(ros::service::call("/gobot_status/charging_status", isCharging) && isCharging.response.isCharging){
            ROS_WARN("(Exploration) we are charging so we go straight to avoid bumping into the CS when turning");
            setSpeed('F', 30, 'F', 30);
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            setSpeed('F', 0, 'F', 0);
        }

        std::thread(doExploration).detach();
        return true;

    } 
    else
        ROS_WARN("(Exploration) We were already exploring");
    return true;

}

/// Service to stop the exploration
bool stopExplorationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(exploring){
        std_msgs::Int8 result;
        result.data=0;
        exploration_pub.publish(result);

        set_gobot_status.request.status = 21;
        set_gobot_status.request.text = "STOP_EXPLORING";
        ros::service::call("/gobot_status/set_gobot_status",set_gobot_status);
    }
    return stopExploration();
}

/// To stop the exploration (either when the service is called or when we are done scanning)
bool stopExploration(void){
    //~ROS_INFO("(Exploration) stopExploration called %d", exploring);
    if(exploring){
        ROS_INFO("(Exploration) Stopped exploring");
        exploring = false;
        ac->cancelAllGoals();
    } 
    else
        ROS_WARN("(Exploration) We were not exploring");
    
    return true;
}

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    //ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR, velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("/gobot_motor/setSpeeds", speed);
}

void timerCallback(const ros::TimerEvent&){
    if(set_gobot_status.request.status==25){
        gobot_msg_srv::SetInt sound_num;
        sound_num.request.data.push_back(2);
        sound_num.request.data.push_back(1);
        ros::service::call("/gobot_base/setSound",sound_num);
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "move_base_controller");
    ros::NodeHandle nh;

    getGobotStatusSrv = nh.serviceClient<gobot_msg_srv::GetGobotStatus>("/gobot_status/get_gobot_status");

    //Startup begin
    ROS_INFO("(Exploration) Waiting for Robot setting hardware...");
    getGobotStatusSrv.waitForExistence(ros::Duration(30.0));
    getGobotStatusSrv.call(get_gobot_status);
    while((get_gobot_status.response.status!=-1 || get_gobot_status.response.text!="STM32_READY") && ros::ok()){
        getGobotStatusSrv.call(get_gobot_status);
        ros::Duration(0.2).sleep();
    }
    ROS_INFO("(Exploration) Robot setting hardware is ready.");
    //Startup end

    ROS_INFO("(Exploration) running...");
    /// Get the data from the parameters server
    nh.param<std::string>("map_frame", map_frame, "map");
    //~ROS_INFO("(Exploration) map_frame : %s", map_frame.c_str());

    nh.param<std::string>("base_frame", base_frame, "base_link");
    //~ROS_INFO("(Exploration) base_frame : %s", base_frame.c_str());

    nh.param<double>("new_goal_frequency", new_goal_frequency, 0.1);
    //~ROS_INFO("(Exploration) new_goal_frequency : %f", new_goal_frequency);

    nh.param<int>("no_frontier_threshold", no_frontier_threshold, 5);
    //~ROS_INFO("(Exploration) no_frontier_threshold : %d", no_frontier_threshold);

    //Publish when exploration is complete
    exploration_pub = nh.advertise<std_msgs::Int8>("/gobot_scan/exploration_result",10);

    /// Create an actionlibClient to be able to send and monitor goals
    ac = std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));

    ros::Timer timer = nh.createTimer(ros::Duration(10), timerCallback);
    /// Launch service's servers
    //0-don't go back to starting point; 1-go back to charging station; 2-go back to normal staring point
    ros::ServiceServer startExploration = nh.advertiseService("/gobot_scan/startExploration", startExplorationSrv);
    ros::ServiceServer stopExploration = nh.advertiseService("/gobot_scan/stopExploration", stopExplorationSrv);

    set_gobot_status.request.status = 20;
    set_gobot_status.request.text = "EXPLORATION";
    ros::service::call("/gobot_status/set_gobot_status",set_gobot_status);

    ros::spin();

    return 0;
}