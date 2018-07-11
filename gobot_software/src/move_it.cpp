#include "gobot_software/move_it.hpp"

#define PLAY_PATH_PORT 8333
#define ROBOT_POS_TOLERANCE 0.05

MoveBaseClient* ac;

// the stage of the robot within the path (if path going from first point to second point, stage is 0, if going from point before last point to last point stage is #points-1)
int stage_ = 0;

// holds whether the robot is ready to accept a new goal or not (already moving towards one)
bool looping = false;
bool dockAfterPath = false;
bool waitingForAction = false, readAction=true, interruptDelay=false, stop_flag=false;

std_srvs::Empty empty_srv;
std::vector<Point> path;
Point currentGoal;
ros::Subscriber button_sub;
ros::Time action_time;
int status_ = 0;
std::string text_ = "";

robot_class::SetRobot SetRobot;
robot_class::GetRobot GetRobot;

void setGobotStatus(int status,std::string text){
	status_ = status;
	text_ = text;
	SetRobot.setStatus(status_,text_);
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
				if(text_=="PLAY_PATH"){
					stage_ = -stage_ - 1;
					SetRobot.setStage(stage_);
				}
				setGobotStatus(0,"ABORTED_PATH");
				SetRobot.setNavSpeed('F', 0, 'F', 0);
				break;
			//OTHER CASE
			default:
				ROS_ERROR("(MOVE_IT) Unknown goal status %d",msg->status.status);
				break;
		}
	}
}


// called when the last goal has been reached
void goalReached(){
	if(text_=="PLAY_POINT"){
		setGobotStatus(0,"COMPLETE_POINT");
	}
	else{
		if(currentGoal.text != "\"\"" && currentGoal.text !=""){   //string "\"\"" means empty
			std::thread tts_thread(textToSpeech,currentGoal.text,currentGoal.delayText);
			tts_thread.detach();
		}
		stage_++;

		if(stage_ >= path.size()){
			//complete path but looping
			if(looping && path.size()>1){
				stage_ = 0;
				if(dockAfterPath){
					//if looping, check the last point delay status before go docking
					interruptDelay = false;
					SetRobot.setSound(1,1); 
					SetRobot.setStage(stage_);
					// reached a normal/path goal so we sleep the given time
					checkGoalDelay();
					std::thread([](){
						//~ROS_INFO("(MOVE_IT::goalReached) battery is low, go to charging station!!");
						ros::service::call("/gobot_command/goDock", empty_srv);
						dockAfterPath = false;
					}).detach();
					return;
				}
			}
			//complete path without looping
			else {
				//ROS_INFO("(MOVE_IT:: complete path!");
				setGobotStatus(0,"COMPLETE_PATH");
				SetRobot.setStage(stage_);
				if(dockAfterPath){
					//if not looping, go dock when reaching last point
					std::thread([](){
						//~ROS_INFO("(MOVE_IT::goalReached) battery is low, go to charging station!!");
						ros::service::call("/gobot_command/goDock", empty_srv);
						dockAfterPath = false;
					}).detach();
				}
				return;
			}
		}

		interruptDelay = false;
		SetRobot.setSound(1,1);
		SetRobot.setStage(stage_);
		// reached a normal/path goal so we sleep the given time
		checkGoalDelay();
		if(!stop_flag){
			setGobotStatus(5,"PLAY_PATH");
			goNextPoint(path.at(stage_));
		}
		else
			stop_flag = false;
	}
}

void checkGoalDelay(){
	if(currentGoal.waitingTime != 0){	
		if(currentGoal.waitingTime > 0){
			setGobotStatus(5,"DELAY");
			//ROS_INFO("(MOVE_IT::goalReached) goalReached going to sleep for %f seconds", currentGoal.waitingTime);
			double dt=0.0;
			ros::Time last_time=ros::Time::now();
			while(dt<currentGoal.waitingTime && !stop_flag && !interruptDelay && ros::ok()){
				dt=(ros::Time::now()-last_time).toSec();
				ros::Duration(0.1).sleep();
				ros::spinOnce();
			}
		}
		else if(currentGoal.waitingTime == -1){
			setGobotStatus(5,"WAITING");
			ros::NodeHandle n;
			//ROS_INFO("(MOVE_IT) Goal reached. Waiting for human action.");
			waitingForAction = true;
			readAction = true;
			button_sub = n.subscribe("/gobot_base/button_topic",1,getButtonCallback);
			while(waitingForAction && !stop_flag && !interruptDelay && ros::ok()){
				ros::Duration(0.1).sleep();
				ros::spinOnce();
			}
			button_sub.shutdown();
		}
	}
}

void textToSpeech(std::string text, double delay){
	ros::Duration(delay).sleep();
	SetRobot.speakChinese(text);
}

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg){
	/// External button 0-No press; 1-press
	if(msg->data==1 && waitingForAction && readAction){
		action_time = ros::Time::now();
		readAction=false;
	}
	else if(msg->data==0 && !readAction){
		readAction = true;
		double dt = (ros::Time::now()-action_time).toSec();
		if(dt<5.0){
			//Go to next point
			waitingForAction=false;
			//user feedback
			SetRobot.setSound(1,1);
			//ROS_INFO("(MOVE_IT) Received Human Action %.2f seconds.",dt);
		}
	}
}

void goNextPoint(const Point _point){
	currentGoal = _point;
	// get the next point in the path list and tell the robot to go there
	//ROS_INFO("(MOVE_IT::goNextPoint) goNextPoint called %d [%f, %f, %f]", stage, currentGoal.x, currentGoal.y,currentGoal.yaw);

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
		ROS_ERROR("(MOVE_IT::goNextPoint) no action server");
}

bool playPointService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
	if (status_==5 && (text_=="DELAY" || text_=="WAITING")){
		stop_flag = true;
	}
	else{
		stop_flag = false;
	}

	setGobotStatus(5,"PLAY_POINT");

	gobot_msg_srv::IsCharging isCharging;
	if(ros::service::call("/gobot_status/charging_status", isCharging) && isCharging.response.isCharging){
		ROS_WARN("(MOVE_IT::playPathService) Go straight because of charging.");
		SetRobot.setNavSpeed('F', 15, 'F', 15);
		ros::Duration(2.5).sleep();
		SetRobot.setNavSpeed('F', 0, 'F', 0);
		SetRobot.setDock(0);
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
	assigned_point.text = "";
	goNextPoint(assigned_point);
	
	return true;
}

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	if(path.empty()){
		ROS_WARN("(MOVE_IT::playPathService) the path is empty, returning false to cmd system");
		return false;
	}

	if (status_==5 && (text_=="DELAY" || text_=="WAITING")){
		stop_flag = true;
	}
	else{
		stop_flag = false;
	}

	if(stage_ < 0){
		stage_ = -stage_ - 1;
		SetRobot.setStage(stage_);
	}
	
	if(stage_ >= path.size()){
		stage_ = 0;
		SetRobot.setStage(stage_);
	}

	setGobotStatus(5,"PLAY_PATH");

	gobot_msg_srv::IsCharging isCharging;
	if(ros::service::call("/gobot_status/charging_status", isCharging) && isCharging.response.isCharging){
		ROS_WARN("(MOVE_IT::playPathService) Go straight because of charging.");
		SetRobot.setNavSpeed('F', 15, 'F', 15);
		ros::Duration(2.5).sleep();
		SetRobot.setNavSpeed('F', 0, 'F', 0);
		SetRobot.setDock(0);
	}

	goNextPoint(path.at(stage_));
	
	return true;	
}

bool updatePathService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
	stage_ = 0;
	SetRobot.setStage(stage_);
	path = std::vector<Point>();
	Point pathPoint;

	int n = 7;
	
	for(int i=0;i<req.data.size();i++){
		//0 is point name, no need to record
		if(i!= 0 && (i-1)%n != 0){
			if((i-1)%n == 1){
				pathPoint.x = std::stod(req.data.at(i));
			} 
			else if((i-1)%n == 2){
				pathPoint.y = std::stod(req.data.at(i));
			} 
			else if((i-1)%n == 3){
				pathPoint.waitingTime = std::stod(req.data.at(i));
				pathPoint.isHome = false;
			} 
			else if((i-1)%n == 4){
				pathPoint.yaw = std::stod(req.data.at(i));
			}
			else if((i-1)%n == 5){
				pathPoint.text = req.data.at(i);
			}
			else if((i-1)%n == 6){
				pathPoint.delayText = req.data.at(i)=="" ? 0.0 : std::stod(req.data.at(i));
				path.push_back(pathPoint);
			}
		}
	}
	ROS_INFO("(MOVE_IT) Updated robot path");
	
	return true;
}

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	//ROS_INFO("(MOVE_IT::stopPathService) stopPathService called");
	setGobotStatus(1,"STOP_PATH");
	stop_flag = true;
	stage_ = 0;
	SetRobot.setStage(stage_);
	if(ac->isServerConnected())
		ac->cancelAllGoals();

    if(dockAfterPath){
		std::thread([](){
			//~ROS_INFO("(MOVE_IT::goalReached) battery is low, go to charging station!!");
			ros::service::call("/gobot_command/goDock", empty_srv);
			dockAfterPath = false;
		}).detach();
	}
	
	return true;
}

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	//~ROS_INFO("(MOVE_IT::pausePathService) pausePathService called");
	if(stage_ < 0){
		stage_ = -stage_-1;
		SetRobot.setStage(stage_);
	}
	setGobotStatus(4,"PAUSE_PATH");
	stop_flag = true;
	if(ac->isServerConnected())
		ac->cancelAllGoals();

	return true;
}

bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(MOVE_IT::startLoopPathService) service called");
	looping = true;
    return SetRobot.setLoop(1);
}

bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(MOVE_IT::stopLoopPathService) service called");
	looping = false;
    return SetRobot.setLoop(0);
}

bool goDockAfterPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	if (status_ == 5){
		ROS_INFO("(MOVE_IT::goDockAfterPathService) robot will auto dock after completing current route.");
		dockAfterPath = true;
		return true;
	}
	else{
		ROS_INFO("(MOVE_IT::goDockAfterPathService) robot will auto dock now.");
		return false;
	}
}

bool interruptDelayService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("(MOVE_IT) Delay interruptted by user");
	interruptDelay = true;
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
	SetRobot.setStage(stage_);

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
	looping = GetRobot.getLoop();
	stage_=GetRobot.getStage();
	path = std::vector<Point>();
	gobot_msg_srv::GetStringArray get_path;
	// we recreate the path to follow from the file
	ros::service::call("/gobot_status/get_path", get_path);
	Point pathPoint;
	int n = 7;
	for(int i=0;i<get_path.response.data.size();i++){
		if(i!= 0 && (i-1)%n != 0){
			if((i-1)%n == 1){
				pathPoint.x=std::stod(get_path.response.data.at(i));
			} 
			else if((i-1)%n == 2){
				pathPoint.y=std::stod(get_path.response.data.at(i));
			} 
			else if((i-1)%n == 3){
				pathPoint.waitingTime=std::stod(get_path.response.data.at(i));
				pathPoint.isHome = false;
			} 
			else if((i-1)%n == 4){
				pathPoint.yaw=std::stod(get_path.response.data.at(i));
			}

			else if((i-1)%n == 5){
				pathPoint.text = get_path.response.data.at(i);
			}
			else if((i-1)%n == 6){
				pathPoint.delayText = get_path.response.data.at(i)=="" ? 0.0 : std::stod(get_path.response.data.at(i));
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
    ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config);

	//Set lower manual speed for scan process
    gobot_msg_srv::SetFloatArray joy_speed;
    joy_speed.request.data.push_back(0.4);
    joy_speed.request.data.push_back(0.8);
    ros::service::call("/gobot_base/set_joy_speed",joy_speed);

}

void mySigintHandler(int sig)
{   
	button_sub.shutdown();
	delete ac;
    ros::shutdown();
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "play_path");
	ros::NodeHandle n;
	signal(SIGINT, mySigintHandler);
	
	SetRobot.initialize();
	
	//Startup begin
	//sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
	ros::service::waitForService("/move_base/TebLocalPlannerROS/set_parameters",ros::Duration(90.0));
    //Startup end
	
	ac = new MoveBaseClient("move_base", true);
	ac->waitForServer(ros::Duration(60.0));

	initData();

	ROS_INFO("(MOVE_IT) play path main running...");
	currentGoal.x = 0;
	currentGoal.y = -1;
	currentGoal.waitingTime = -1;
	currentGoal.isHome = false;
	currentGoal.text = "";
	currentGoal.delayText = 0.0;

	// service to interrupt delay/human action
	ros::ServiceServer _interruptDelayService 	= n.advertiseService("/gobot_function/interrupt_delay", interruptDelayService);
	// service to play the robot's point
	ros::ServiceServer _pointPathService 		= n.advertiseService("/gobot_function/play_point", playPointService);
	// service to play the robot's path
	ros::ServiceServer _playPathService 		= n.advertiseService("/gobot_function/play_path", playPathService);
	// service to pause the robot's path
	ros::ServiceServer _pausePathService 		= n.advertiseService("/gobot_function/pause_path", pausePathService);
	// service to stop the robot's path
	ros::ServiceServer _stopPathService 		= n.advertiseService("/gobot_function/stop_path", stopPathService);
	// service to delete path
	ros::ServiceServer _updatePathService 		= n.advertiseService("/gobot_function/update_path", updatePathService);
	// service to skip the path point
	ros::ServiceServer _skipPath 				= n.advertiseService("/gobot_function/skip_path", skipPathService);
	// service to make the path loop
	ros::ServiceServer _startLoopPath 			= n.advertiseService("/gobot_function/startLoopPath", startLoopPathService);
	// service to stop the path loop
	ros::ServiceServer _stopLoopPath 			= n.advertiseService("/gobot_function/stopLoopPath", stopLoopPathService);
	// the battery is low so we need to go dock after finishing our path
	ros::ServiceServer _goDockAfterPath 		= n.advertiseService("/gobot_function/goDockAfterPath", goDockAfterPathService);
	// tell the action client that we want to spin a thread by default

	// get the current status of the goal 
	ros::Subscriber goalResult = n.subscribe("/move_base/result",1,goalResultCallback);

	ros::spin();

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
