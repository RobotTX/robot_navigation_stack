#include <gobot_recovery/scan_startup.h>
#include <ctime>

ros::Publisher vel_pub;
std_srvs::Empty empty_srv;

bool buttonOn=true;
ros::Time action_time;
std::string map_path,map_id;

robot_class::SetRobot SetRobot;
robot_class::GetRobot GetRobot;
int robot_status_;
std::string status_text_;

std::string getCurrentTime(){
  std::time_t now = std::time(0);
  std::stringstream transTime;
  transTime << std::put_time(localtime(&now), "%F-%H-%M-%S");
  return transTime.str();
}

void getButtonCallback(const std_msgs::Int8::ConstPtr& msg){
  // External button 0-No press; 1-press
  if(msg->data==1 && buttonOn){
		action_time = ros::Time::now();
		buttonOn=false;
	}
	else if(msg->data==0 && !buttonOn){
    GetRobot.getStatus(robot_status_);
		buttonOn = true;
    double dt = (ros::Time::now() - action_time).toSec();
    if(dt<=5.0){
      //start exploration
      if(robot_status_==21){
        ROS_INFO("(SCAN_STARTUP) Continue robot exploration.");
        ros::service::call("/gobot_command/start_explore",empty_srv);
      }
      //stop exploring
      else if(robot_status_==25){
        ROS_INFO("(SCAN_STARTUP) Stop robot exploration.");
        ros::service::call("/gobot_command/stop_explore",empty_srv);
      }
    }
    else if(dt>5.0){
      //exploration startup, then start auto-exploration
      if(robot_status_==20){
        ROS_INFO("SCAN_STARTUP) Start robot exploration.");
        ros::service::call("/gobot_command/start_explore",empty_srv);
      }
      //save map
      else if(robot_status_==21){
        //user feedback
        SetRobot.setSound(1,1);
        saveMap();
      }
    }
	}
}

void saveMap(){
  ROS_INFO("(SCAN_STARTUP) Save scaned map.");
  std::string cmd = "rosrun map_server map_saver -f "+ map_path +" &";
  system(cmd.c_str());
  std::time_t now = time(0);

  std::srand(std::time(nullptr));  // use current time as seed for random generator
  std::string mapDate = getCurrentTime();
  std::string mapId = "{9abcdef"+ std::to_string(std::rand()%10) +"-123"+ std::to_string(std::rand()%10) +"-456"+ std::to_string(std::rand()%10) 
                        +"-789"+ std::to_string(std::rand()%10) +"-123456789ab"+ std::to_string(std::rand()%10) +"}"; 

  std::ofstream ofs(map_id, std::ofstream::out | std::ofstream::trunc);
  if(ofs){
      ofs << mapId << std::endl << mapDate << std::endl;
      ofs.close();
      ROS_INFO("(SCAN_STARTUP) map id: %s.",mapId.c_str());
  }

  SetRobot.setHome("0","0","0","0","0","1");
  ROS_INFO("(SCAN_STARTUP) Home deleted");

  SetRobot.setLoop(0);
  ROS_INFO("(SCAN_STARTUP) Loop deleted");

  /// We delete the old path
  SetRobot.clearPath();
  ROS_INFO("(SCAN_STARTUP) Path deleted");

  /// We delete the old path stage
  SetRobot.setStage(0);
  ROS_INFO("(SCAN_STARTUP) Path stage deleted");
}

bool saveMapSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  saveMap();
  return true;
}


bool poseReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  return true;
}

void mySigintHandler(int sig)
{

  ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_startup");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    
    SetRobot.initialize();
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(SCAN_STARTUP) Waiting for Robot setting hardware...");
    ros::service::waitForService("/gobot_startup/sensors_ready", ros::Duration(60.0));
    ros::service::waitForService("/move_base/clear_costmaps", ros::Duration(60.0));
    ROS_INFO("(SCAN_STARTUP) Robot setting hardware is ready.");
    
    SetRobot.setBatteryLed();
    SetRobot.setStatus(-3,"SCAN_READY");
    
    ros::ServiceServer poseReadySrv = nh.advertiseService("/gobot_startup/pose_ready", poseReadySrvCallback);
    //Startup end
    
    nh.getParam("map_id_file", map_id);
    nh.getParam("map_path", map_path);
    ROS_INFO("(SCAN_STARTUP) map_path: %s.",map_path.c_str());
    
    ros::ServiceServer saveMapSrv = nh.advertiseService("/gobot_scan/save_map", saveMapSrvCallback);

    ros::Subscriber button_sub = nh.subscribe("/gobot_base/button_topic",1,getButtonCallback);
    
    ros::spin();
    return 0;
}