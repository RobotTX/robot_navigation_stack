#include <gobot_recovery/startup.h>

std_srvs::Empty empty_srv;

int count = 1;
bool buttonOn=true;
ros::Time action_time;
robot_class::GetRobot GetRobot;
int robot_status_;
std::string status_text_;

void initialPoseResultCallback(const std_msgs::Int8::ConstPtr& msg){
  if(msg->data==0){
    ROS_ERROR("(NAV_STARTUP) Can not find robot pose when start up. Retry after 10 sec.");
    ROS_ERROR("(NAV_STARTUP) Or move robot to charging station and restart.");
    if(count<3){
      ROS_ERROR("(NAV_STARTUP) Tried %d times",count);
      ros::Duration(10.0).sleep();
      while(!ros::service::call("/gobot_recovery/globalize_pose",empty_srv) && ros::ok()){
          ROS_ERROR("(NAV_STARTUP) Failed to start robot");
          ros::service::call("/gobot_recovery/stop_globalize_pose",empty_srv);
          ros::Duration(1.0).sleep();
      }
      count++;
    }
    else{
      ROS_ERROR("(NAV_STARTUP) Give up! Move robot to charging station and restart.");
    }
  }
}


void getButtonCallback(const std_msgs::Int8::ConstPtr& msg){
  // External button 1-No press; 0-press
  if(msg->data==0 && buttonOn){
		action_time = ros::Time::now();
		buttonOn=false;
	}
	else if(msg->data==1 && !buttonOn){
    GetRobot.getStatus(robot_status_,status_text_);
		buttonOn = true;
    double dt = (ros::Time::now() - action_time).toSec();
    if(dt<=5.0){
      //play path when pause, complete or stop path/docking
      if(robot_status_==4 || robot_status_==0 || robot_status_==1 || (robot_status_==11 && status_text_=="STOP_DOCKING")){
        ROS_INFO("(NAV_STARTUP) Continue robot path.");
        ros::service::call("/gobot_command/play_path",empty_srv);
      }
      //pause path when play path
      else if(robot_status_==5 && status_text_!="WAITING"){
        ROS_INFO("(NAV_STARTUP) Pause robot path.");
        ros::service::call("/gobot_command/pause_path",empty_srv);
      }
      //if go docking, stop it
      else if(robot_status_==15){
        ROS_INFO("(NAV_STARTUP) Stop robot home.");
        ros::service::call("/gobot_command/stopGoDock",empty_srv);
      }
    }
    else if(dt>5.0 && dt<=10.0){
      //if robot pause, reset the path
      if(robot_status_==4){
        ROS_INFO("(NAV_STARTUP) Reset robot path.");
        ros::service::call("/gobot_command/stop_path",empty_srv);
      }

      ROS_INFO("(NAV_STARTUP) Start robot path.");
      //play the path
      ros::service::call("/gobot_command/play_path",empty_srv);
      
    }
    else if(dt>10.0 && dt<=20.0){
      ROS_INFO("(NAV_STARTUP) Send robot home.");
      ros::service::call("/gobot_command/goDock",empty_srv);
		}
	}
}

void mySigintHandler(int sig)
{

  ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "startup");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(NAV_STARTUP) Waiting for Robot setting hardware...");
    ros::service::waitForService("/gobot_startup/sensors_ready", ros::Duration(60.0));
    ROS_INFO("(NAV_STARTUP) Robot setting hardware is ready.");
    //Startup end
    
    ros::Subscriber initialPoseResult = nh.subscribe("/gobot_recovery/find_initial_pose",1, initialPoseResultCallback);
    ros::Subscriber button_sub = nh.subscribe("/gobot_base/button_topic",1,getButtonCallback);

    ros::service::waitForService("/move_base/clear_costmaps", ros::Duration(60.0));
    ros::service::waitForService("/request_nomotion_update", ros::Duration(60.0));
    ros::service::waitForService("/gobot_recovery/initialize_pose", ros::Duration(60.0));
    ros::service::call("/gobot_recovery/initialize_pose",empty_srv);
    ros::spin();
    return 0;
}