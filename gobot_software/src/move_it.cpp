#include "gobot_software/move_it.hpp"

#define PLAY_PATH_PORT 8333
#define ROBOT_POS_TOLERANCE 0.05

std::shared_ptr<MoveBaseClient> ac(0);

// the stage of the robot within the path (if path going from first point to second point, stage is 0, if going from point before last point to last point stage is #points-1)
int stage_ = 0;

// holds whether the robot is ready to accept a new goal or not (already moving towards one)
bool looping = false;
bool dockAfterPath = false;
bool waitingForAction = false, readAction=true,stop_flag=false;

std_srvs::Empty empty_srv;
std::vector<Point> path;
Point currentGoal;
ros::Subscriber button_sub;
ros::Time action_time;
int status_ = 0;
std::string text_ = "";

robot_class::SetRobot robot;

void setStageInFile(const int stage){
	robot.setStage(stage);
}

void setGobotStatus(int status,std::string text){
	status_ = status;
	text_ = text;
	robot.setStatus(status_,text_);
}

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
	if(status_==5 && (text_=="PLAY_PATH" || text_=="PLAY_POINT")){
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
				setStageInFile(-stage_ - 1);
				setGobotStatus(0,"ABORTED_PATH");
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
	stage_ = text_=="PLAY_PATH" ? stage_+1:stage_;
	if(stage_ >= path.size() && text_!="PLAY_POINT"){
		if(looping && !dockAfterPath && path.size()>1){
			//ROS_INFO("(PlayPath:: complete path but looping!!");
			stage_ = 0;
		}
		else{
			//ROS_INFO("(PlayPath:: complete path!");
			setGobotStatus(0,"COMPLETE_PATH");
			setStageInFile(stage_);
			if(dockAfterPath){
				//ROS_INFO("(PlayPath::goalReached) battery is low, go to charging station!!");
				ros::service::call("/gobot_command/goDock", empty_srv);
				dockAfterPath = false;
			}
			return;
		}
	}
	robot.setSound(1,1); 
	// reached a normal/path goal so we sleep the given time
	checkGoalDelay();
	if(!stop_flag){
		setGobotStatus(5,"PLAY_PATH");
		setStageInFile(stage_);
		goNextPoint(path.at(stage_));
	}
	else
		stop_flag = false;
}

void checkGoalDelay(){
	if(currentGoal.waitingTime != 0)
	{
		if(currentGoal.waitingTime > 0){
			setGobotStatus(5,"DELAY");
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
			setGobotStatus(5,"WAITING");
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

void goNextPoint(const Point _point){
	currentGoal = _point;
	// get the next point in the path list and tell the robot to go there
	//ROS_INFO("(PlayPath::goNextPoint) goNextPoint called %d [%f, %f, %f]", stage, currentGoal.x, currentGoal.y,currentGoal.yaw);

	// sends the next goal to the robot
	move_base_msgs::MoveBaseGoal goal;
	tf::Quaternion quaternion;
	quaternion.setRPY(0, 0, -(currentGoal.yaw+90)*3.14159/180);

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

bool savePointService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
	
	return true;
}

bool playPointService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
	if (status_==5 && (text_=="DELAY" || text_=="WAITING")){
		stop_flag = true;
		if (stage_>=0){
			if(stage_==0)
				stage_ = path.size()-1;
			else
				stage_ = stage_ - 1;
		}
	}
	else{
		stop_flag = false;
	}

	if(stage_>=path.size()){
		stage_ = 0;
	}
	
	setStageInFile(-99);

	gobot_msg_srv::IsCharging arg;
	if(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging){
		ROS_WARN("(PlayPath::playPathService) we are charging so we go straight to avoid bumping into the CS when turning");
		robot.setMotorSpeed('F', 15, 'F', 15);
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		robot.setMotorSpeed('F', 0, 'F', 0);
	}

	//request data: 0-name,1-x,2-y,3-orientation, 4-home or not
	Point assigned_point;
	assigned_point.x = std::stod(req.data[1]);
	assigned_point.y = std::stod(req.data[2]);
	assigned_point.yaw = std::stod(req.data[3]);
	//if charging station, go ahead of it
	if(req.data[4]=="1"){
		assigned_point.x = assigned_point.x + 0.4 * std::cos(assigned_point.yaw);
		assigned_point.y = assigned_point.y + 0.4 * std::sin(assigned_point.yaw);
	}
	assigned_point.isHome = false;
	assigned_point.waitingTime = -1;
	setGobotStatus(5,"PLAY_POINT");
	goNextPoint(assigned_point);
	
	return true;
}

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	if(path.empty()){
		ROS_ERROR("(PlayPath::playPathService) the path is empty, returning false to cmd system");
		return false;
	}

	if (status_==5 && (text_=="DELAY" || text_=="WAITING")){
		stop_flag = true;
	}
	else{
		stop_flag = false;
	}

	if(stage_>=path.size()){
		stage_ = 0;
		setStageInFile(stage_);
	}

	gobot_msg_srv::IsCharging arg;
	if(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging){
		ROS_WARN("(PlayPath::playPathService) we are charging so we go straight to avoid bumping into the CS when turning");
		robot.setMotorSpeed('F', 15, 'F', 15);
		std::this_thread::sleep_for(std::chrono::milliseconds(2500));
		robot.setMotorSpeed('F', 0, 'F', 0);
	}
	setGobotStatus(5,"PLAY_PATH");
	goNextPoint(path.at(stage_));
	
	return true;	
}

bool updatePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	stage_ = 0;
	setStageInFile(stage_);
	path = std::vector<Point>();
	gobot_msg_srv::GetStringArray get_path;
	// we recreate the path to follow from the file
	if(ros::service::call("/gobot_status/get_path", get_path)){
		Point pathPoint;
		for(int i=0;i<get_path.response.data.size();i++){
			if(i!= 0 && (i-1)%5 != 0){
        		if((i-1)%5 == 1){
        			pathPoint.x=std::stod(get_path.response.data.at(i));
        		} 
				else if((i-1)%5 == 2){
        			pathPoint.y=std::stod(get_path.response.data.at(i));
        		} 
				else if((i-1)%5 == 3){
        			pathPoint.waitingTime=std::stod(get_path.response.data.at(i));
		            pathPoint.isHome = false;
        		} 
				else if((i-1)%5 == 4){
					pathPoint.yaw=std::stod(get_path.response.data.at(i));
					path.push_back(pathPoint);
				}
        	}
		}
	}
	
	return true;
}

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	//ROS_INFO("(PlayPath::stopPathService) stopPathService called");
	setGobotStatus(1,"STOP_PATH");
	stop_flag = true;
	stage_ = 0;
	setStageInFile(stage_);
	ros::service::call("/move_base/clear_costmaps",empty_srv);
	if(ac->isServerConnected())
		ac->cancelAllGoals();

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
	setStageInFile(stage_);
	if(ac->isServerConnected())
		ac->cancelAllGoals();

	if(dockAfterPath){
		std::thread([](){
			//~ROS_INFO("(PlayPath::goalReached) battery is low, go to charging station!!");
			ros::service::call("/gobot_command/goDock", empty_srv);
			dockAfterPath = false;
		}).detach();
	}

	return true;
}

bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(PlayPath::startLoopPathService) service called");
	looping = true;
    return robot.setLoop(1);
}

bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(PlayPath::stopLoopPathService) service called");
	looping = false;
    return robot.setLoop(0);
}

bool goDockAfterPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(PlayPath::goDockAfterPathService) service called");
    dockAfterPath = true;
    return true;
}

bool skipPathService(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
	if (path.empty()){
		return false;
	}
	//if go to/reached the first point and backward path,set it to be the last point
	if(stage_==0 && req.data==-1){
		stage_ = path.size()-1;
	}
	//if go to/reached the last point forward path,set it to be the first point
	else if(stage_>=path.size()-1 && req.data==1){
		stage_ = 0;
	}
	else{
		stage_ +=req.data;
	}
	setStageInFile(stage_);

	//if robot is waiting for interaction, quit it
	if (status_==5){
		if((text_=="DELAY_" || text_=="WAITING")){
			stop_flag = true;
		}
		else{
			stop_flag = false;
		}
		setGobotStatus(5,"PLAY_PATH");
		goNextPoint(path.at(stage_));
	}
	return true;
}

void initData(){
	gobot_msg_srv::GetInt get_loop;
	ros::service::call("/gobot_status/get_loop",get_loop);
	looping = get_loop.response.data;

	gobot_msg_srv::GetInt get_stage;
	ros::service::call("/gobot_status/get_stage", get_stage);
	stage_=get_stage.response.data;

	path = std::vector<Point>();
	gobot_msg_srv::GetStringArray get_path;
	// we recreate the path to follow from the file
	ros::service::call("/gobot_status/get_path", get_path);
	Point pathPoint;
	for(int i=0;i<get_path.response.data.size();i++){
		if(i!= 0 && (i-1)%5 != 0){
			if((i-1)%5 == 1){
				pathPoint.x=std::stod(get_path.response.data.at(i));
			} 
			else if((i-1)%5 == 2){
				pathPoint.y=std::stod(get_path.response.data.at(i));
			} 
			else if((i-1)%5 == 3){
				pathPoint.waitingTime=std::stod(get_path.response.data.at(i));
				pathPoint.isHome = false;
			} 
			else if((i-1)%5 == 4){
				pathPoint.yaw=std::stod(get_path.response.data.at(i));
				path.push_back(pathPoint);
			}
		}
	}

	//Set speed given by user
	gobot_msg_srv::GetStringArray get_speed;
	ros::service::call("/gobot_status/get_speed",get_speed);
	dynamic_reconfigure::Reconfigure config;
    dynamic_reconfigure::DoubleParameter linear_param,angular_param;
    linear_param.name = "max_vel_x";
    linear_param.value = std::stod(get_speed.response.data[0]);
    angular_param.name = "max_vel_theta";
    angular_param.value = std::stod(get_speed.response.data[1])*3.14159/180;

    config.request.config.doubles.push_back(linear_param);
    config.request.config.doubles.push_back(angular_param);
	ros::service::waitForService("/move_base/TebLocalPlannerROS/set_parameters",ros::Duration(30));
    ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config);

}

void mySigintHandler(int sig)
{   
    setStageInFile(stage_);
    ros::shutdown();
}

int main(int argc, char* argv[]){

	ROS_INFO("(PlayPath) play path main running...");

	try {

		ros::init(argc, argv, "play_path");
		ros::NodeHandle n;

		signal(SIGINT, mySigintHandler);
		
		ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(60.0));
		
		initData();

		currentGoal.x = 0;
		currentGoal.y = -1;
		currentGoal.waitingTime = -1;
		currentGoal.isHome = false;

		// service to save the robot's point
		ros::ServiceServer _savePointPathService = n.advertiseService("/gobot_function/save_point", savePointService);
		// service to play the robot's point
		ros::ServiceServer _pointPathService = n.advertiseService("/gobot_function/play_point", playPointService);
		// service to play the robot's path
		ros::ServiceServer _playPathService = n.advertiseService("/gobot_function/play_path", playPathService);
		// service to pause the robot's path
		ros::ServiceServer _pausePathService = n.advertiseService("/gobot_function/pause_path", pausePathService);
		// service to stop the robot's path
		ros::ServiceServer _stopPathService = n.advertiseService("/gobot_function/stop_path", stopPathService);
		// service to delete path
		ros::ServiceServer _updatePathService = n.advertiseService("/gobot_function/update_path", updatePathService);
		// service to skip the path point
        ros::ServiceServer _skipPath = n.advertiseService("/gobot_function/skip_path", skipPathService);
        // service to make the path loop
        ros::ServiceServer _startLoopPath = n.advertiseService("/gobot_function/startLoopPath", startLoopPathService);
        // service to stop the path loop
        ros::ServiceServer _stopLoopPath = n.advertiseService("/gobot_function/stopLoopPath", stopLoopPathService);
        // the battery is low so we need to go dock after finishing our path
        ros::ServiceServer _goDockAfterPath = n.advertiseService("/gobot_function/goDockAfterPath", goDockAfterPathService);
		// tell the action client that we want to spin a thread by default
	
		// get the current status of the goal 
		ros::Subscriber goalResult = n.subscribe("/move_base/result",1,goalResultCallback);

		ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));
		ac->waitForServer();

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
