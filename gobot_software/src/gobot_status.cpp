#include <ros/ros.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <gobot_msg_srv/SetGobotStatus.h>
#include <string>
#include <mutex>
#include <gobot_msg_srv/SetDockStatus.h>
#include <gobot_msg_srv/GetDockStatus.h>


#define CHARGING
#define DOCKING 15
#define WAITING 10
#define PLAY_PATH 5
#define PAUSE_PATH 4
#define STOP_PATH 1
#define STOP_DOCKING 1
#define COMPLETE_PATH 0
#define FREE -99

std::mutex statusMutex1,statusMutex2;

int gobot_status=-99;
std::string gobot_text = "FREE";

//3->charging 1->go to docking 0->not charging -1->failed to docking
int dock_status = 0;


bool setDockStatus(gobot_msg_srv::SetDockStatus::Request &req, gobot_msg_srv::SetDockStatus::Response &res){
    statusMutex2.lock();
    dock_status = req.status;
    statusMutex2.unlock();
    ROS_INFO("Dock status: %d", dock_status);

    return true;
}

bool getDockStatus(gobot_msg_srv::GetDockStatus::Request &req, gobot_msg_srv::GetDockStatus::Response &res){
    statusMutex2.lock();
    res.status = dock_status;
    statusMutex2.unlock();

    return true;
}


bool getGobotStatus(gobot_msg_srv::GetGobotStatus::Request &req, gobot_msg_srv::GetGobotStatus::Response &res){
    statusMutex1.lock();
    res.status = gobot_status;
    res.text = gobot_text;
    statusMutex1.unlock();
    return true;
}



bool setGobotStatus(gobot_msg_srv::SetGobotStatus::Request &req, gobot_msg_srv::SetGobotStatus::Response &res){
    statusMutex1.lock();
    gobot_status = req.status;
    gobot_text = req.text;
    statusMutex1.unlock();
    ROS_INFO("Gobot status: %d,%s",gobot_status,gobot_text.c_str());
    return true;
}



int main(int argc, char* argv[]){

    try{
        ros::init(argc, argv, "gobot_status");
        ros::NodeHandle n;


        ros::ServiceServer setGobotStatusSrv = n.advertiseService("/gobot_status/get_gobot_status", getGobotStatus);
        ros::ServiceServer getGobotStatusSrv = n.advertiseService("/gobot_status/set_gobot_status", setGobotStatus);

        ros::ServiceServer setDockStatusSrv = n.advertiseService("/gobot_status/setDockStatus", setDockStatus);
        ros::ServiceServer getDockStatusSrv = n.advertiseService("/gobot_status/getDockStatus", getDockStatus);

        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("(Gobot_status system) Exception : %s", e.what());
    }

    return 0;
}
