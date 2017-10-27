#include <gobot_recovery/scan_startup.h>

ros::Publisher vel_pub;
std_srvs::Empty empty_srv;

bool buttonOn=true;
ros::Time action_time;
gobot_msg_srv::GetGobotStatus get_gobot_status;
ros::ServiceClient getGobotStatusSrv;
std::string map_path,map_id;
std::string mapId, mapDate;

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
      //start exploration
      if(get_gobot_status.response.status==21){
        ROS_INFO("Continue robot exploration.");
        ros::service::call("/gobot_command/start_explore",empty_srv);
      }
      //stop exploring
      else if(get_gobot_status.response.status==25){
        ROS_INFO("Stop robot exploration.");
        ros::service::call("/gobot_command/stop_explore",empty_srv);
      }
    }
    else if(dt>5.0 && dt<=10.0){
      //exploration
      if(get_gobot_status.response.status==20){
        ROS_INFO("Start robot exploration.");
        ros::service::call("/gobot_command/start_explore",empty_srv);
      }
      //save map
      else if(get_gobot_status.response.status==21){
        ROS_INFO("Save scaned map.");
        std::string cmd = "rosrun map_server map_saver -f "+ map_path +" &";
        system(cmd.c_str());
        
        int id = std::rand() % 10000;
        mapDate = "1990-12-05-00-00-00";
        mapId = "{b09f0a3b-b0da-4c50-aa66-ff0b3080"+std::to_string(id)+"}";

        std::ofstream ofs(map_id, std::ofstream::out | std::ofstream::trunc);
        if(ofs){
            ofs << mapId << std::endl << mapDate << std::endl;
            ofs.close();
            ROS_INFO("(startup) map id: %s.",mapId.c_str());
        }
      }
    }
    else if(dt>10.0){
      //save map
      ROS_INFO("Save scaned map.");
      std::string cmd = "rosrun map_server map_saver -f "+ map_path +" &";
      system(cmd.c_str());

      int n = std::rand() % 1000;
      mapDate = "1990-12-05-00-00-00";
      mapId = "{b09f0a3b-b0da-4c50-aa66-ff0b30807"+std::to_string(n)+"}";

      std::ofstream ofs(map_id, std::ofstream::out | std::ofstream::trunc);
      if(ofs){
          ofs << mapId << std::endl << mapDate << std::endl;
          ofs.close();
          ROS_INFO("(startup) map id: %s.",mapId.c_str());
      }
    }
	}
}

void mySigintHandler(int sig)
{

  ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_startup");
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
    nh.getParam("map_path", map_path);
    ROS_INFO("(startup) map_path: %s.",map_path.c_str());

    nh.getParam("map_id_file", map_id);
    
    ros::Subscriber button_sub = nh.subscribe("/gobot_base/button_topic",1,getButtonCallback);
    
    ROS_INFO("Started Robot.");

    ros::spin();
    return 0;
}