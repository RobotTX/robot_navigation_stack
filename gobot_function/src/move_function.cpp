#include "gobot_function/move_function.h"

#define PLAY_PATH_PORT 8333

//the stage of the robot within the path (if path going from first point to second point, stage is 0, 
//if going from point before last point to last point stage is #points-1)
int stage_ = 0;
bool looping = false, dockAfterPath = false, audioOn = false;
bool waitingForAction = false, readAction=true, interruptDelay=false, stop_flag=false, delayOn=false;
std::vector<PathPoint> path_;
PathPoint current_goal_;
int status_ = 0;
std::string text_ = "", audio_ack_ = "!@#$.mp3";
std::string audio_folder_;

std_srvs::Empty empty_srv;
ros::Subscriber button_sub;
ros::Time action_time;

robot_class::RobotMoveClass MoveRobot;
robot_class::GetRobot GetRobot;

//****************************** CALLBACK ******************************
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
	MoveRobot.getStatus(status_,text_);
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
					MoveRobot.setStage(stage_);
				}
				MoveRobot.setStatus(0,"ABORTED_PATH");
				MoveRobot.setNavSpeed('F', 0, 'F', 0);
				break;
			//OTHER CASE
			default:
				ROS_ERROR("(MOVE_IT) Unknown goal status %d",msg->status.status);
				break;
		}
	}
}


void getButtonCallback(const std_msgs::Int8::ConstPtr& msg){
	/// External button 0-No press; 1-press
	if(msg->data==1 && waitingForAction && readAction){
		action_time = ros::Time::now();
		readAction=false;
	}
	else if(msg->data==0 && !readAction){
		readAction = true;
		if((ros::Time::now()-action_time).toSec()<5.0){
			waitingForAction=false;
			MoveRobot.setSound(1,1);
			//ROS_INFO("(MOVE_IT) Received Human Action %.2f seconds.",dt);
		}
	}
}

//****************************** SERVICE ******************************
bool playPointService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
	stop_flag = delayOn ? true : false;

	//request data: 0-name,1-x,2-y,3-orientation, 4-home or not
	PathPoint point;
	point.p.x = std::stod(req.data[1]);
	point.p.y = std::stod(req.data[2]);
	point.p.yaw = MoveRobot.appToRobotYaw(std::stod(req.data[3]));
	//if charging station, go ahead of it
	if(req.data[4]=="1"){
		point.p.x += 0.4 * std::cos(point.p.yaw);
		point.p.y += 0.4 * std::sin(point.p.yaw);
	}
	point.is_home = false;
	point.delay_point = -1;
	point.text = "";
	point.audio_index = -1;
	
	MoveRobot.setStatus(5,"PLAY_POINT");

	MoveRobot.moveFromCS();

	moveToGoal(point);
	
	return true;
}

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	if(path_.empty()){
		ROS_ERROR("(MOVE_IT::playPathService) the path is empty");
		return false;
	}

	stop_flag = delayOn ? true : false;

	if(stage_ < 0){
		stage_ = -stage_ - 1;
		MoveRobot.setStage(stage_);
	}
	else if(stage_ >= path_.size()){
		stage_ = 0;
		MoveRobot.setStage(stage_);
	}

	MoveRobot.setStatus(5,"PLAY_PATH");

	MoveRobot.moveFromCS();

	moveToGoal(path_.at(stage_));

	return true;	
}

bool updatePathService(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
	stage_ = 0;
	MoveRobot.setStage(stage_);

	path_.clear();
	readPath(req.data);

	ROS_INFO("(MOVE_IT) Updated robot path");
	
	return true;
}

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	stop_flag = true;

	MoveRobot.cancelMove();

	if(audioOn){
		MoveRobot.killAudio();
		audioOn = false;
	}

	//ROS_INFO("(MOVE_IT::stopPathService) stopPathService called");
	MoveRobot.setStatus(1,"STOP_PATH");

	stage_ = 0;
	MoveRobot.setStage(stage_);

    if(dockAfterPath){
		goDockAfterPath();
	}
	
	return true;
}

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	stop_flag = true;

	MoveRobot.cancelMove();

	if(audioOn){
		MoveRobot.killAudio();
		audioOn = false;
	}

	MoveRobot.setStatus(4,"PAUSE_PATH");

	if(stage_ < 0){
		stage_ = -stage_-1;
		MoveRobot.setStage(stage_);
	}

	return true;
}

bool startLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	looping = true;
    return MoveRobot.setLoop(1);
}

bool stopLoopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	looping = false;
    return MoveRobot.setLoop(0);
}

bool goDockAfterPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	MoveRobot.getStatus(status_,text_);
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
	ROS_INFO("(MOVE_IT::interruptDelayService) Delay interruptted by user");
	interruptDelay = true;
	return true;
}


//NOT IN USE
bool skipPathService(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
	if (path_.empty()){
		return false;
	}
	//if go to/reached the first point and backward path,set it to be the last point
	if(stage_==0 && req.data==-1){
		stage_ = path_.size()-1;
	}
	//if go to/reached the last point forward path,set it to be the first point
	else if(stage_>=path_.size()-1 && req.data==1){
		stage_ = 0;
	}
	else{
		stage_ +=req.data;
	}
	MoveRobot.setStage(stage_);

	//if robot is waiting for interaction, quit it
	MoveRobot.getStatus(status_,text_);
	if (delayOn){
		if((text_=="DELAY_" || text_=="WAITING")){
			stop_flag = true;
		}
		else{
			stop_flag = false;
		}
		MoveRobot.setStatus(5,"PLAY_PATH");

		moveToGoal(path_.at(stage_));
	}
	return true;
}


//****************************** FUNCTIONS ******************************
void goalReached(){
	if(text_=="PLAY_POINT"){
		MoveRobot.setStatus(0,"COMPLETE_POINT");
		return;
	}

	checkSound();

	stage_++;

	if(stage_ >= path_.size()){
		//complete path with looping
		if(looping && path_.size()>1){
			stage_ = 0;
			if(dockAfterPath){
				//if looping, check the last point delay status before go docking
				MoveRobot.setSound(1,1); 
				MoveRobot.setStage(stage_);
				// reached a normal/path goal so we sleep the given time
				checkGoalDelay();
				goDockAfterPath();
				return;
			}
		}
		//complete path without looping
		else {
			MoveRobot.setStatus(0,"COMPLETE_PATH");
			MoveRobot.setStage(stage_);
			if(dockAfterPath){
				goDockAfterPath();
			}
			return;
		}
	}

	MoveRobot.setSound(1,1);
	MoveRobot.setStage(stage_);

	// reached a normal/path goal so we sleep the given time
	checkGoalDelay();
	if(!stop_flag){
		MoveRobot.setStatus(5,"PLAY_PATH");
		moveToGoal(path_.at(stage_));
	}
	else
		stop_flag = false;
}

void moveToGoal(PathPoint goal){
	current_goal_ = goal;
	MoveRobot.moveTo(current_goal_.p);
}

void checkSound(){
	audioOn = false;
	if(current_goal_.text != "\"\"" && current_goal_.text !=""){   //string "\"\"" means empty
		if(audio_ack_.compare(current_goal_.text)==0 && current_goal_.audio_index>=0){
			audioOn = true;
			//robot will complete audio before conituning path 
			if(current_goal_.delay_sound == 999){
				playAudio(audio_folder_+std::to_string(current_goal_.audio_index)+".mp3", 0);
			}
			else{
				std::thread play_audio(playAudio, audio_folder_+std::to_string(current_goal_.audio_index)+".mp3", current_goal_.delay_sound);
				play_audio.detach();
			}
		}
		else{
			std::thread tts_thread(textToSpeech, current_goal_.text, current_goal_.delay_sound);
			tts_thread.detach();
		}
	}
}

void checkGoalDelay(){
	delayOn = true;
	interruptDelay = false;
	if(current_goal_.delay_point != 0){	
		if(current_goal_.delay_point > 0){
			MoveRobot.setStatus(5,"DELAY");
			//ROS_INFO("(MOVE_IT::goalReached) goalReached going to sleep for %f seconds", current_goal_.delay_point);
			double dt=0.0;
			ros::Time last_time=ros::Time::now();
			while(dt<current_goal_.delay_point && !stop_flag && !interruptDelay && ros::ok()){
				dt=(ros::Time::now()-last_time).toSec();
				ros::Duration(0.1).sleep();
				ros::spinOnce();
			}
		}
		else if(current_goal_.delay_point == -1){
			MoveRobot.setStatus(5,"WAITING");
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
	delayOn = false;
}


void textToSpeech(std::string text, double delay){
	ros::Duration(delay).sleep();
	MoveRobot.speakChinese(text);
}

void playAudio(std::string file, double delay){
	ros::Duration(delay).sleep();
	if(GetRobot.getMute() == 0){
		std::string audio_file = "sudo play " + file;
		system(audio_file.c_str());
	}
}

void readPath(std::vector<std::string> &path){
	PathPoint pathPoint;
	int n = 7, audio_index = 0;
	for(int i=1;i<path.size();i++){
		switch((i-1)%n){
			case 1:
				pathPoint.p.x=std::stod(path.at(i));
				break;

			case 2:
				pathPoint.p.y=std::stod(path.at(i));
				break;

			case 3:
				pathPoint.delay_point=std::stod(path.at(i));
				pathPoint.is_home = false;
				break;

			case 4:
				pathPoint.p.yaw=MoveRobot.appToRobotYaw(std::stod(path.at(i)));
				break;

			case 5:
				if (audio_ack_.compare(path.at(i))==0){
					pathPoint.audio_index = audio_index;
					audio_index++;
				}
				else{
					pathPoint.audio_index = -1;
				}
				pathPoint.text = path.at(i);
				break;

			case 6:
				pathPoint.delay_sound = path.at(i)=="" ? 0.0 : std::stod(path.at(i));
				path_.push_back(pathPoint);
				break;

			default:
				break;
		}
	}
}

void goDockAfterPath(){
	std::thread([](){
		dockAfterPath = false;
		ros::service::call("/gobot_command/goDock", empty_srv);
	}).detach();
}

void initData(){
	ros::NodeHandle n;
	n.getParam("audio_folder", audio_folder_);

	looping = GetRobot.getLoop();
	stage_=GetRobot.getStage();
	gobot_msg_srv::GetStringArray get_path;
	// we recreate the path to follow from the file
	ros::service::call("/gobot_status/get_path", get_path);
	readPath(get_path.response.data);

	//Set speed given by user
	gobot_msg_srv::GetStringArray get_speed;
	ros::service::call("/gobot_status/get_speed",get_speed);
	MoveRobot.setAutoSpeedLimit(std::stod(get_speed.response.data[0]),MoveRobot.degreeToRad(std::stod(get_speed.response.data[1])));

	//Set lower manual speed for scan process
	MoveRobot.setManualSpeedLimit(0.4,0.8);
}

void mySigintHandler(int sig)
{   
    ros::shutdown();
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "move_function");
	ros::NodeHandle n;
	signal(SIGINT, mySigintHandler);

	//Startup begin
	//sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(60.0));
	ros::service::waitForService("/move_base/TebLocalPlannerROS/set_parameters",ros::Duration(60.0));
    //Startup end

	MoveRobot.moveClientInitialize();

	initData();

	ROS_INFO("(MOVE_IT) play path main running...");

	// get the current status of the goal 
	ros::Subscriber goalResult = n.subscribe("/move_base/result",1,goalResultCallback);

	ros::ServiceServer interruptDelaySrv 	= n.advertiseService("/gobot_function/interrupt_delay", interruptDelayService);
	ros::ServiceServer pointPathSrv			= n.advertiseService("/gobot_function/play_point", 		playPointService);
	ros::ServiceServer playPathSrv 			= n.advertiseService("/gobot_function/play_path", 		playPathService);
	ros::ServiceServer pausePathSrv 		= n.advertiseService("/gobot_function/pause_path", 		pausePathService);
	ros::ServiceServer stopPathSrv			= n.advertiseService("/gobot_function/stop_path", 		stopPathService);
	ros::ServiceServer updatePathSrv		= n.advertiseService("/gobot_function/update_path", 	updatePathService);
	ros::ServiceServer skipPathSrv			= n.advertiseService("/gobot_function/skip_path", 		skipPathService);
	ros::ServiceServer startLoopPathSrv		= n.advertiseService("/gobot_function/startLoopPath", 	startLoopPathService);
	ros::ServiceServer stopLoopPathSrv 		= n.advertiseService("/gobot_function/stopLoopPath", 	stopLoopPathService);
	ros::ServiceServer goDockAfterPathSrv 	= n.advertiseService("/gobot_function/goDockAfterPath", goDockAfterPathService);

	ros::spin();

	return 0;
}
