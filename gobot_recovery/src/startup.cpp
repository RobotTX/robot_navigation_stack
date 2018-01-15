#include <gobot_recovery/startup.h>

ros::Publisher vel_pub;
std_srvs::Empty empty_srv;

int count = 1;
bool buttonOn=true;
ros::Time action_time;
std::string restart_sh;
gobot_msg_srv::GetGobotStatus get_gobot_status;
ros::ServiceClient getGobotStatusSrv;

void initialPoseResultCallback(const std_msgs::Int8::ConstPtr& msg){
  if(msg->data==0){
    ROS_ERROR("Can not find robot pose when start up. Retry after 10 sec.");
    ROS_ERROR("Or move robot to charging station and restart.");
    if(count<3){
      ROS_ERROR("Tried %d times",count);
      ros::Duration(10.0).sleep();
      while(!ros::service::call("/gobot_recovery/globalize_pose",empty_srv) && ros::ok()){
          ROS_ERROR("Failed to start robot");
          ros::service::call("/gobot_recovery/stop_globalize_pose",empty_srv);
          ros::Duration(1.0).sleep();
      }
      count++;
    }
    else{
      ROS_ERROR("Give up! Move robot to charging station and restart.");
    }
  }
}


void getButtonCallback(const std_msgs::Int8::ConstPtr& msg){
  if(msg->data==0 && buttonOn){
		action_time = ros::Time::now();
		buttonOn=false;
	}
	else if(msg->data==1 && !buttonOn){
    getGobotStatusSrv.call(get_gobot_status);
		buttonOn = true;
    double dt = (ros::Time::now() - action_time).toSec();
    if(dt<=5.0){
      //play path
      if(get_gobot_status.response.status==4){
        ROS_INFO("Continue robot path.");
        ros::service::call("/gobot_command/play_path",empty_srv);
      }
      //pause path
      else if(get_gobot_status.response.status==5 && get_gobot_status.response.text!="WAITING"){
        ROS_INFO("Pause robot path.");
        ros::service::call("/gobot_command/pause_path",empty_srv);
      }
      //if go docking, stop it
      else if(get_gobot_status.response.status==15){
        ROS_INFO("Stop robot home.");
        ros::service::call("/gobot_command/stopGoDock",empty_srv);
      }
    }
    else if(dt>5.0 && dt<=10.0){
      //if robot pause, reset the path
      if(get_gobot_status.response.status==4){
        ROS_INFO("Reset robot path.");
        ros::service::call("/gobot_command/stop_path",empty_srv);
      }

      if(get_gobot_status.response.status!=5){
        ROS_INFO("Start robot path.");
        //play the path
        ros::service::call("/gobot_command/play_path",empty_srv);
      }
      
    }
    else if(dt>10.0 && dt<=15.0){
      ROS_INFO("Send robot home.");
      ros::service::call("/gobot_command/goDock",empty_srv);
		}
    /*
    //These two cases are not considered now
    else if(dt>99.0 && dt<=999.0){
      ROS_INFO("Globalize robot.");
      ros::service::call("/gobot_recovery/globalize_pose",empty_srv);
    }
		else if(dt>999.0){
			//restart
      ROS_INFO("Restart robot. Please wait for xx seconds...");
			const std::string restart_script = "sh " + restart_sh + " &";
      system(restart_script.c_str());
		}
    */
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

    getGobotStatusSrv = nh.serviceClient<gobot_msg_srv::GetGobotStatus>("/gobot_status/get_gobot_status");
    
    //Startup begin
    ROS_INFO("(startup) Waiting for Robot setting hardware...");
    getGobotStatusSrv.waitForExistence(ros::Duration(30.0));
    getGobotStatusSrv.call(get_gobot_status);
    while((get_gobot_status.response.status!=-1 || get_gobot_status.response.text!="STM32_READY") && ros::ok()){
        getGobotStatusSrv.call(get_gobot_status);
        ros::Duration(0.2).sleep();
    }
    ROS_INFO("(startup) Robot setting hardware is ready.");
    //Startup end

    ROS_INFO("Starting Robot...");
    
    nh.getParam("restart_file", restart_sh);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    ros::Subscriber initialPoseResult = nh.subscribe("/gobot_recovery/find_initial_pose",1, initialPoseResultCallback);
    ros::Subscriber button_sub = nh.subscribe("/gobot_base/button_topic",1,getButtonCallback);

    ros::service::waitForService("/gobot_recovery/initialize_pose", ros::Duration(30.0));
    while(!ros::service::call("/gobot_recovery/initialize_pose",empty_srv) && ros::ok()){
        ROS_ERROR("Failed to initilize robot pose");
        ros::service::call("/gobot_recovery/stop_globalize_pose",empty_srv);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Started Robot.");

    ros::spin();
    return 0;
}