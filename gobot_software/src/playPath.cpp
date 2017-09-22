#include "playPath.hpp"

#define PLAY_PATH_PORT 8333
#define ROBOT_POS_TOLERANCE 0.05

std::shared_ptr<MoveBaseClient> ac(0);

// the stage of the robot within the path (if path going from first point to second point, stage is 0, if going from point before last point to last point stage is #points-1)
int stage = 0;

// holds whether the robot is ready to accept a new goal or not (already moving towards one)
bool waitingForNextGoal = false;
bool looping = false;
bool dockAfterPath = false;

std::vector<Point> path;
Point currentGoal;


ros::Subscriber statusSuscriber;
ros::Subscriber sub_robot;


void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg){
	// if there is a currentGoal
	if(currentGoal.x != -1){
		/// we check if the robot is close enough to its goal
		if(std::abs(msg->position.x - currentGoal.x) < ROBOT_POS_TOLERANCE && std::abs(msg->position.y - currentGoal.y) < ROBOT_POS_TOLERANCE){
			/// if the robot has already arrived, we want to wait for the next goal instead of repeating the same "success" functions
			if(!waitingForNextGoal){
				ROS_INFO("(PlayPath::getRobotPos) robot close enough to the goal");
				ROS_INFO("(PlayPath::getRobotPos) robot position [%f, %f]", msg->position.x, msg->position.y);
				ROS_INFO("(PlayPath::getRobotPos) robot goal [%f, %f]", currentGoal.x, currentGoal.y);
				waitingForNextGoal = true;
				goalReached();
			}
		}
	}
}

// to get the status of the robot (completion of the path towards its next goal, SUCCEEDED = reached its goal, ACTIVE = currently moving towards its goal)
void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray){
	if(currentGoal.x != -1){
		if(goalStatusArray->status_list[0].status == 3){
			// if we reached the goal
			if(!waitingForNextGoal){
                ROS_INFO("(PlayPath::getStatus) robot close enough to the goal");
                waitingForNextGoal = true;
				goalReached();
			}
		} else if(goalStatusArray->status_list[0].status == 4 || goalStatusArray->status_list[0].status == 5){
			// if the goal could not be reached
			if(!waitingForNextGoal){
				waitingForNextGoal = true;
				// we use -stage to tell where on the path we blocked
				// - 1 because if we get stuck going to the first stage, it's going to be 0
				setStageInFile(-stage - 1);
			}
		} else {
			waitingForNextGoal = false;
		}
	}
}

// called when the last goal has been reached
void goalReached(){
	if(currentGoal.isHome){
		ROS_INFO("(PlayPath::goalReached) home reached");
	} else {
		ROS_INFO("(PlayPath::goalReached) path point reached");
		stage++;
		if(stage >= path.size()){
			// the robot successfully reached all its goals
			ROS_INFO("(PlayPath::goalReached) I have completed my journey Master Joda, what will you have me do ?");
            if(dockAfterPath){
                ROS_INFO("(PlayPath::goalReached) Battery is low, go to charging station!!");

                // resets the stage of the path to be able to play the path from the start again
                stage = 10000;
                // resets the current goal
                currentGoal.x = -1;
                setStageInFile(stage);

                std_srvs::Empty arg;
                if(!ros::service::call("goDock", arg))
                    ROS_ERROR("(PlayPath::goalReached) Could not go charging");

                dockAfterPath = false;

            } else if(looping){
                ROS_INFO("(PlayPath::goalReached) Looping!!");
                stage = 0;
                setStageInFile(stage);
                goNextPoint();
            } else {
                // resets the stage of the path to be able to play the path from the start again
                stage = 10000;
                // resets the current goal
                currentGoal.x = -1;
                setStageInFile(stage);
            }
		} else {
			// reached a normal/path goal so we sleep the given time
			if(currentGoal.waitingTime > 0){
				ROS_INFO("(PlayPath::goalReached) goalReached going to sleep for %f seconds", currentGoal.waitingTime);
                setStageInFile(stage);
				sleep(currentGoal.waitingTime);
			}
			goNextPoint();
		}
	}
}

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("(PlayPath::stopPathService) stopPathService called");
	/// if action server is up -> cancel
	if(ac->isServerConnected())
		ac->cancelAllGoals();
	currentGoal.x = -1;
	stage = 0;
	setStageInFile(stage);

    if(dockAfterPath){
        ROS_INFO("(PlayPath::goalReached) Battery is low, go to charging station!!");
        
        std_srvs::Empty arg;
        if(!ros::service::call("goDock", arg))
            ROS_ERROR("(PlayPath::goalReached) Could not go charging");

        dockAfterPath = false;

    }
	return true;
}

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("(PlayPath::pausePathService) pausePathService called");
	if(ac->isServerConnected())
		ac->cancelAllGoals();


    if(dockAfterPath){
        ROS_INFO("(PlayPath::goalReached) Battery is low, go to charging station!!");
        
        std_srvs::Empty arg;
        if(!ros::service::call("goDock", arg))
            ROS_ERROR("(PlayPath::goalReached) Could not go charging");

        dockAfterPath = false;

    }
    
	return true;
}

void goNextPoint(){

	// get the next point in the path list and tell the robot to go there
	if(path.size()-1 == stage)
		ROS_INFO("(PlayPath::goNextPoint) This is my final destination");

	ROS_INFO("(PlayPath::goNextPoint) goNextPoint called %d [%f, %f]", stage, path.at(stage).x, path.at(stage).y);

    Point point;
    point.x = path.at(stage).x;
    point.y = path.at(stage).y;
    point.waitingTime = path.at(stage).waitingTime;
    point.isHome = false;

    goToPoint(point);
}

// sends the next goal to the robot
void goToPoint(const Point& point){
	ROS_INFO("(PlayPath::goToPoint) goToPoint [%f, %f]", point.x, point.y);
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = point.x;
    goal.target_pose.pose.position.y = point.y;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1;

	currentGoal = point;
	if(ac->isServerConnected()) ac->sendGoal(goal);
	else ROS_INFO("(PlayPath::goToPoint) no action server");
}

void setStageInFile(const int _stage){

	std::string pathStageFile;
	ros::NodeHandle n;
	if(n.hasParam("path_stage_file")){
		n.getParam("path_stage_file", pathStageFile);
		ROS_INFO("(PlayPath::setStageInFile) set path stage file to %s", pathStageFile.c_str());
	}
	std::ofstream path_stage_file(pathStageFile, std::ios::out | std::ios::trunc);
	// when the stage is < 0 it means the robot was blocked at stage -stage (which is > 0)
	if(path_stage_file){
		ROS_INFO("(PlayPath::setStageInFile) setStageInFile %d", _stage);
		path_stage_file << _stage;
		path_stage_file.close();
	} else {
		ROS_INFO("(PlayPath::setStageInFile) Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played");
	}
}

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("(PlayPath::playPathService) called while at stage : %d", stage);

	path = std::vector<Point>();

	ros::NodeHandle n;
	std::string pathFile;
	if(n.hasParam("path_file")){
		n.getParam("path_file", pathFile);
		ROS_INFO("(PlayPath::playPathService) set path file to %s", pathFile.c_str());
	}
	std::ifstream file(pathFile, std::ios::in);

	// we recreate the path to follow from the file
	if(file){

        std::string line;

        int i = 0;
        Point pathPoint;
        while(getline(file, line)){
        	if(i!= 0 && (i-1)%4 != 0){
	        	std::istringstream iss(line);
        		if((i-1)%4 == 1){
        			iss >> pathPoint.x;
        		} else if((i-1)%4 == 2){
        			iss >> pathPoint.y;
        		} else if((i-1)%4 == 3){
        			iss >> pathPoint.waitingTime;
		            pathPoint.isHome = false;
		            path.push_back(pathPoint);
        		}
        	}
            i++;
        }

	} else {
		ROS_ERROR("(PlayPath::playPathService) sorry could not find the path file on the robot, returning false to the cmd system");
		return false;
	}

	if(path.empty()){
		ROS_ERROR("(PlayPath::playPathService) the path is empty, returning false to cmd system");
		return false;
	}

	for(size_t i = 0; i < path.size(); i++)
		ROS_INFO("(PlayPath::playPathService) Stage %lu [%f, %f], wait %f sec", i, path.at(i).x, path.at(i).y, path.at(i).waitingTime);


	if(currentGoal.x == -1){
		stage = 0;
		setStageInFile(stage);
	}

	goNextPoint();

	return true;	
}

bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("(PlayPath::startLoopPathService) service called");
    looping = true;
    return true;
}

bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("(PlayPath::stopLoopPathService) service called");
    looping = false;
    return true;
}

bool goDockAfterPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("(PlayPath::goDockAfterPathService) service called");
    looping = false;
    dockAfterPath = true;
    return true;
}

int main(int argc, char* argv[]){

	ROS_INFO("(PlayPath) play path main running...");

	try {

		ros::init(argc, argv, "play_path");

		currentGoal.x = -1;
		currentGoal.y = -1;
		currentGoal.waitingTime = -1;
		currentGoal.isHome = false;
		
		ros::NodeHandle n;

		// service to play the robot's path
		ros::ServiceServer _playPathService = n.advertiseService("play_path", playPathService);

		// service to pause the robot's path
		ros::ServiceServer _pausePathService = n.advertiseService("pause_path", pausePathService);

		// service to stop the robot's path
		ros::ServiceServer _stopPathService = n.advertiseService("stop_path", stopPathService);

        // service to make the path loop
        ros::ServiceServer _startLoopPath = n.advertiseService("startLoopPath", startLoopPathService);

        // service to stop the path loop
        ros::ServiceServer _stopLoopPath = n.advertiseService("stopLoopPath", stopLoopPathService);

        // the battery is low so we need to go dock after finishing our path
        ros::ServiceServer _goDockAfterPath = n.advertiseService("goDockAfterPath", goDockAfterPathService);

		// tell the action client that we want to spin a thread by default
		ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));

		// get the current status of the goal 
		statusSuscriber = n.subscribe("/move_base/status", 1, getStatus);

		// get the position of the robot to compare with the goal
		sub_robot = n.subscribe("/robot_pose", 1, getRobotPos);

		// wait for the action server to come up
		while(!ac->waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for the move_base action server to come up");

		if(ac->isServerConnected())
                        ROS_INFO("Play path:: Server is connected");

		ros::spin();

	} catch (std::exception& e) {
		ROS_ERROR("(Play path) Exception : %s", e.what());
	}

	return 0;
}

/*

uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
*/
