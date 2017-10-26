#include "gobot_software/playPath.hpp"

#define PLAY_PATH_PORT 8333
#define ROBOT_POS_TOLERANCE 0.05

std::shared_ptr<MoveBaseClient> ac(0);

// the stage of the robot within the path (if path going from first point to second point, stage is 0, if going from point before last point to last point stage is #points-1)
int stage = 0;

// holds whether the robot is ready to accept a new goal or not (already moving towards one)
bool looping = false;
bool dockAfterPath = false;
bool waitingForAction = false, readAction=true,stop_flag=false;

std_srvs::Empty empty_srv;
std::vector<Point> path;
Point currentGoal;
ros::Subscriber button_sub;
ros::ServiceClient setGobotStatusSrv;
ros::Time action_time;

gobot_msg_srv::SetGobotStatus set_gobot_status;

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    ////ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR, velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("/gobot_motor/setSpeeds", speed);
}

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
	if(set_gobot_status.request.status==5 && set_gobot_status.request.text=="PLAY_PATH"){
		switch(msg->status.status){
			//PREEMTED
			case 2:
				break;
			//SUCCEED
			case 3:
				goalReached();
				break;
			//ABORTED
			case 4:
				setStageInFile(-stage - 1);
				setGobotStatus(0,"ABORTED_PATH");
				//set the UI button
				ros::service::call("/gobot_command/pause_path",empty_srv);
				break;
			//REJECTED
			case 5:
				setStageInFile(-stage - 1);
				setGobotStatus(0,"ABORTED_PATH");
				//set the UI button
				ros::service::call("/gobot_command/pause_path",empty_srv);
				break;
			//OTHER CASE
			default:
				ROS_ERROR("Unknown goal status %d",msg->status.status);
				break;
		}
	}
}

// called when the last goal has been reached
void goalReached(){
	//ROS_INFO("(PlayPath::goalReached) path point reached");
	stage++;
	setStageInFile(stage);
	if(stage >= path.size()){
		if(looping && !dockAfterPath && path.size()>1){
			//ROS_INFO("(PlayPath:: complete path but looping!!");
			stage = 0;
			setStageInFile(stage);
			checkGoalDelay();
			if(!stop_flag)
				goNextPoint();
		}
		else{
			//ROS_INFO("(PlayPath:: complete path!");
			setGobotStatus(0,"COMPLETE_PATH");
			// resets the current goal
			currentGoal.x = -1;
			//set the UI button
			ros::service::call("/gobot_command/pause_path",empty_srv);

			if(dockAfterPath){
				//ROS_INFO("(PlayPath::goalReached) battery is low, go to charging station!!");
				ros::service::call("/gobot_command/goDock", empty_srv);
				dockAfterPath = false;
			}
		}
	} 
	else {
		// reached a normal/path goal so we sleep the given time
		checkGoalDelay();
		if(!stop_flag)
			goNextPoint();
	}
}

void checkGoalDelay(){
	if(currentGoal.waitingTime != 0)
	{
		setGobotStatus(5,"WAITING");

		if(currentGoal.waitingTime > 0){
			//ROS_INFO("(PlayPath::goalReached) goalReached going to sleep for %f seconds", currentGoal.waitingTime);
			double dt=0.0;
			ros::Time last_time=ros::Time::now();
			while(dt<currentGoal.waitingTime && !stop_flag && ros::ok()){
				dt=(ros::Time::now()-last_time).toSec();
				ros::Duration(0.1).sleep();
				ros::spinOnce();
			}
		}
		else if(currentGoal.waitingTime == -1){
			ros::NodeHandle n;
			//ROS_INFO("Goal reached. Waiting for human action.");
			waitingForAction=true;
			button_sub = n.subscribe("/gobot_base/button_topic",1,getButtonCallback);
			while(waitingForAction && !stop_flag && ros::ok()){
				ros::Duration(0.1).sleep();
				ros::spinOnce();
			}
			button_sub.shutdown();
		}
	}
}

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg){
	/// External button 1-No press; 0-press
	if(msg->data==0 && waitingForAction && readAction){
		action_time = ros::Time::now();
		readAction=false;
	}
	else if(msg->data==1 && !readAction){
		readAction = true;
		double dt = (ros::Time::now() - action_time).toSec();
		if(dt<=10.0){
			//Go to next point
			waitingForAction=false;
			//ROS_INFO("Received Human Action %.2f seconds.",dt);
		}
	}
}

void goNextPoint(){
	currentGoal = path.at(stage);
	setGobotStatus(5,"PLAY_PATH");
	// get the next point in the path list and tell the robot to go there
	//ROS_INFO("(PlayPath::goNextPoint) goNextPoint called %d [%f, %f, %f]", stage, currentGoal.x, currentGoal.y,currentGoal.yaw);

	stop_flag=false;

	// sends the next goal to the robot
	move_base_msgs::MoveBaseGoal goal;
	tf::Quaternion quaternion;
	quaternion.setEuler(0, 0, -(currentGoal.yaw+90)*3.14159/180);

	goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = currentGoal.x;
    goal.target_pose.pose.position.y = currentGoal.y;
    goal.target_pose.pose.orientation.x = quaternion.x();
    goal.target_pose.pose.orientation.y = quaternion.y();
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();

	if(ac->isServerConnected()) 
		ac->sendGoal(goal);
	else 
		ROS_ERROR("(PlayPath::goNextPoint) no action server");
}

void setStageInFile(const int _stage){
	gobot_msg_srv::SetStage set_stage;
	set_stage.request.stage = _stage;
	ros::service::call("/gobot_status/set_stage", set_stage);
}

void setGobotStatus(int status,std::string text){
	set_gobot_status.request.status = status;
	set_gobot_status.request.text = text;
	setGobotStatusSrv.call(set_gobot_status);
}

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	//ROS_INFO("(PlayPath::playPathService) called while at stage : %d", stage);

	path = std::vector<Point>();

	gobot_msg_srv::GetPath get_path;
	// we recreate the path to follow from the file
	if(ros::service::call("/gobot_status/get_path", get_path)){
		Point pathPoint;
		for(int i=0;i<get_path.response.path.size();i++){
			if(i!= 0 && (i-1)%5 != 0){
        		if((i-1)%5 == 1){
        			pathPoint.x=std::stod(get_path.response.path.at(i));
        		} 
				else if((i-1)%5 == 2){
        			pathPoint.y=std::stod(get_path.response.path.at(i));
        		} 
				else if((i-1)%5 == 3){
        			pathPoint.waitingTime=std::stod(get_path.response.path.at(i));
		            pathPoint.isHome = false;
        		} 
				else if((i-1)%5 == 4){
					pathPoint.yaw=std::stod(get_path.response.path.at(i));
					path.push_back(pathPoint);
				}
        	}
		}
	} 
	else{
		ROS_ERROR("(PlayPath::playPathService) sorry could not find the path");
		return false;
	}

	if(path.empty()){
		ROS_ERROR("(PlayPath::playPathService) the path is empty, returning false to cmd system");
		return false;
	}

	for(size_t i = 0; i < path.size(); i++)
		//ROS_INFO("(PlayPath::playPathService) Stage %lu [%f, %f, %f], wait %f sec", i, path.at(i).x, path.at(i).y, path.at(i).yaw, path.at(i).waitingTime);

	if(currentGoal.x == -1 || stage==path.size()){
		stage = 0;
		setStageInFile(stage);
	}

	gobot_msg_srv::IsCharging arg;
	if(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging){
		ROS_WARN("(PlayPath::playPathService) we are charging so we go straight to avoid bumping into the CS when turning");
		setSpeed('F', 25, 'F', 25);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		setSpeed('F', 0, 'F', 0);
	}
	goNextPoint();

	return true;	
}


bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	//ROS_INFO("(PlayPath::stopPathService) stopPathService called");

	setGobotStatus(1,"STOP_PATH");

	ros::service::call("/move_base/clear_costmaps",empty_srv);
	currentGoal.x = -1;
	stage = 0;
	setStageInFile(stage);
	
    if(dockAfterPath){
		std::thread([](){
			//~ROS_INFO("(PlayPath::goalReached) battery is low, go to charging station!!");
			ros::service::call("/gobot_command/goDock", empty_srv);
			dockAfterPath = false;
		}).detach();
	}
	
	return true;
}

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	//~ROS_INFO("(PlayPath::pausePathService) pausePathService called");

	setGobotStatus(4,"PAUSE_PATH");

	stop_flag = true;
	if(ac->isServerConnected())
		ac->cancelAllGoals();

	return true;
}

bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(PlayPath::startLoopPathService) service called");
	looping = true;
	gobot_msg_srv::SetInt set_loop;
	set_loop.request.data.push_back(1);
	ros::service::call("/gobot_status/set_loop",set_loop);
    return true;
}

bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(PlayPath::stopLoopPathService) service called");
	looping = false;
	gobot_msg_srv::SetInt set_loop;
	set_loop.request.data.push_back(0);
	ros::service::call("/gobot_status/set_loop",set_loop);
    return true;
}

bool goDockAfterPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(PlayPath::goDockAfterPathService) service called");
    dockAfterPath = true;
    return true;
}

bool skipPathService(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
	//if go to/reached the first point and backward path,set it to be the last point
	if(stage==0 && req.data[0]==-1){
		stage = path.size()-1;
	}
	//if go to/reached the last point forward path,set it to be the first point
	else if(stage>=path.size()-1 && req.data[0]==1){
		stage = 0;
	}
	else{
		stage +=req.data[0];
	}
	setStageInFile(stage);

	gobot_msg_srv::GetGobotStatus get_gobot_status;
    ros::service::call("/gobot_status/get_gobot_status",get_gobot_status);
	//if robot is playing path
	if (get_gobot_status.response.status==5){
		//if robot is waiting for interaction, quit it
		if(get_gobot_status.response.text=="WAITING"){
			stop_flag=true;
		}
		goNextPoint();
	}
	//if robot is not playing path, just change goal state
	else{
		currentGoal.x = 0;
	}
	return true;
}

int main(int argc, char* argv[]){

	ROS_INFO("(PlayPath) play path main running...");

	try {

		ros::init(argc, argv, "play_path");
		ros::NodeHandle n;
		
		currentGoal.x = 0;
		currentGoal.y = -1;
		currentGoal.waitingTime = -1;
		currentGoal.isHome = false;

		// service to play the robot's path
		ros::ServiceServer _playPathService = n.advertiseService("/gobot_function/play_path", playPathService);
		// service to pause the robot's path
		ros::ServiceServer _pausePathService = n.advertiseService("/gobot_function/pause_path", pausePathService);
		// service to stop the robot's path
		ros::ServiceServer _stopPathService = n.advertiseService("/gobot_function/stop_path", stopPathService);
		// service to skip the path point
        ros::ServiceServer _skipPath = n.advertiseService("/gobot_function/skip_path", skipPathService);
        // service to make the path loop
        ros::ServiceServer _startLoopPath = n.advertiseService("/gobot_function/startLoopPath", startLoopPathService);
        // service to stop the path loop
        ros::ServiceServer _stopLoopPath = n.advertiseService("/gobot_function/stopLoopPath", stopLoopPathService);
        // the battery is low so we need to go dock after finishing our path
        ros::ServiceServer _goDockAfterPath = n.advertiseService("/gobot_function/goDockAfterPath", goDockAfterPathService);
		// tell the action client that we want to spin a thread by default
		
		setGobotStatusSrv = n.serviceClient<gobot_msg_srv::SetGobotStatus>("/gobot_status/set_gobot_status");

		ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));	
		// get the current status of the goal 
		ros::Subscriber goalResult = n.subscribe("/move_base/result",1,goalResultCallback);

		// wait for the action server to come up
		while(!ac->waitForServer(ros::Duration(10.0)))
			//~ROS_INFO("Waiting for the move_base action server to come up");

		if(ac->isServerConnected())
            ROS_INFO("Play path Action lib server is connected");
		

		ros::service::waitForService("/gobot_status/get_loop", ros::Duration(30));
		gobot_msg_srv::GetInt get_loop;
		ros::service::call("/gobot_status/get_loop",get_loop);
		looping = get_loop.response.data[0];

		gobot_msg_srv::GetStage get_stage;
		ros::service::call("/gobot_status/get_stage", get_stage);
		stage=get_stage.response.stage;

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
