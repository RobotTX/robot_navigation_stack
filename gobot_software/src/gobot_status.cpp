#include <ros/ros.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <string>
#include <mutex>

#define CHARGING
#define DOCKING 15
#define WAITING 10
#define PLAY_PATH 5
#define PAUSE_PATH 4
#define STOP_PATH 1
#define STOP_DOCKING 1
#define COMPLETE_PATH 0
#define FREE -99

std::mutex msgMutex;

int gobot_status=-99;
std::string gobot_text = "FREE";

bool getGobotStatus(gobot_msg_srv::GetGobotStatus::Request &req, gobot_msg_srv::GetGobotStatus::Response &res){
    msgMutex.lock();
    res.status = gobot_status;
    res.text = gobot_text;
    msgMutex.unlock();
    return true;
}



bool setGobotStatus(gobot_msg_srv::SetGobotStatus::Request &req, gobot_msg_srv::SetGobotStatus::Response &res){
    msgMutex.lock();
    gobot_status = req.status;
    gobot_text = req.text;
    msgMutex.unlock();
    ROS_INFO("Gobot status: %d,%s",gobot_status,gobot_text.c_str());
    return true;
}



int main(int argc, char* argv[]){

    try{
        ros::init(argc, argv, "gobot_status");
        ros::NodeHandle n;


        ros::ServiceServer setGobotStatusSrv = n.advertiseService("/gobot_status/get_gobot_status", getGobotStatus);
        ros::ServiceServer getGobotStatusSrv = n.advertiseService("/gobot_status/set_gobot_status", setGobotStatus);

        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("(Gobot_status system) Exception : %s", e.what());
    }

    return 0;
}
