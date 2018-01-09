#include <gobot_software/gobot_status.hpp>

/*
GOBOT STATUS
25 EXPLORING
21 STOP_EXPLORING/COMPLETE_EXPLORING
20 EXPLORATION
15 DOCKING
11 STOP_DOCKING/FAIL_DOKCING
5  PLAY_PATH/WAITING/DELAY
4  PAUSE_PATH
1  STOP_PATH
0  COMPLETE_PATH/ABORTED_PATH
-1 STM32_READY/FOUND_POSE

DOCK STATUS
3  GO TO CHARGING
1  CHARING
0  NOT CHARGING
-1 FAILED TO GO TO CHARGING
*/

std::mutex gobotStatusMutex,dockStatusMutex,stageMutex,pathMutex,nameMutex,homeMutex,loopMutex,muteMutex,wifiMutex,batteryMutex;
std_srvs::Empty empty_srv;

std::string wifiFile, deleteWifi;
std::vector<std::string> wifi_;

std::string muteFile;
int mute_ = 0;

int gobot_status_=-99;
std::string gobot_text_ = "FREE";

//3->go to docking 1->charging 0->not charging -1->failed to docking
int dock_status_ = 0;

std::string pathStageFile;
int stage_ = 0;

std::string pathLoopFile;
int loop_ = 0;

std::string lowBatteryFile;
std::string low_battery_="0.0";

std::string pathFile;
std::vector<std::string> path_;

std::string nameFile;
std::string hostname_="Go_Gobot";

std::string homeFile;
std::vector<std::string> home_(6,"0");

std::string disconnectedFile;
int disconnected = 0;

bool disconnectedSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    disconnected++;
    std::ofstream ofsDisconnected(disconnectedFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsDisconnected){
        ofsDisconnected << disconnected;
    }

    return true;
}

bool setWifiSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
    if(req.data[0]=="" && req.data[1]==""){
        std::thread([](){
            gobot_msg_srv::SetInt sound_num;
            sound_num.request.data.push_back(1);
            sound_num.request.data.push_back(2);
            ros::service::call("/gobot_base/setSound",sound_num);
        }).detach();
    }
    
    if(wifi_.at(0)==req.data[0] && wifi_.at(1)==req.data[1]){
        return false;
    }
    //if previous wifi is not empty and receive a new wifi, we delete the previous wifi in the netowrk list
    if(wifi_.at(0)!="" && (wifi_.at(0)!=req.data[0] || wifi_.at(1)!=req.data[1]))
    {   
        //delete previously connected wifi
        const std::string deleteWIFI_script = "sudo sh " + deleteWifi + " " + wifiFile ;
        system(deleteWIFI_script.c_str());
    }
    wifiMutex.lock();
    wifi_.clear();
    for(int i=0;i<req.data.size();i++)
        wifi_.push_back(req.data[i]);
    wifiMutex.unlock();

    std::ofstream ofsWifi(wifiFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsWifi){
        for(int i = 0; i < wifi_.size(); i++){
            ofsWifi << wifi_.at(i) << "\n";
        }
        ofsWifi.close();
        ROS_INFO("(Gobot_status) Set Gobot wifi: name:%s, password:%s",wifi_.at(0).c_str(),wifi_.at(1).c_str());
    }
    wifiMutex.unlock();

    return true;
}

bool getWifiSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    wifiMutex.lock();
    for(int i=0;i<wifi_.size();i++)
        res.data.push_back(wifi_.at(i));
    wifiMutex.unlock();
    return true;
}


bool setHomeSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
    homeMutex.lock();
    home_.clear();
    for(int i=0;i<req.data.size();i++)
        home_.push_back(req.data[i]);
    homeMutex.unlock();

    std::ofstream ofsHome(homeFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsHome){
        ofsHome << home_.at(0) << " " << home_.at(1) << " " << home_.at(2) << " " << home_.at(3) << " " << home_.at(4) << " " << home_.at(5);
        ofsHome.close();
        ROS_INFO("(Gobot_status) set Gobot home: [%f, %f] [%f, %f, %f, %f]", std::stod(home_.at(0)),std::stod(home_.at(1)),std::stod(home_.at(2)),std::stod(home_.at(3)),std::stod(home_.at(4)),std::stod(home_.at(5)));
    }

    return true;
}

bool getHomeSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    homeMutex.lock();
    for(int i=0;i<home_.size();i++)
        res.data.push_back(home_.at(i));
    homeMutex.unlock();
    return true;
}


bool setBatterySrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
    batteryMutex.lock();
    low_battery_=req.data[0];
    batteryMutex.unlock();

    std::ofstream ofsBattery(lowBatteryFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsBattery){
        ofsBattery << low_battery_;
        ofsBattery.close();
        ROS_INFO("(Gobot_status) set Gobot battery: %.2f", std::stod(low_battery_));
    }

    return true;
}

bool getBatterySrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    batteryMutex.lock();
    res.data.push_back(low_battery_);
    batteryMutex.unlock();
    return true;
}

bool setNameSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
    nameMutex.lock();
    hostname_=req.data[0];
    nameMutex.unlock();

    std::ofstream ofsName(nameFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsName){
        ofsName << hostname_;
        ofsName.close();
        ROS_INFO("(Gobot_status) Set Gobot name: %s",hostname_.c_str());
    }
    ros::service::call("/gobot_software/update_status",empty_srv);

    return true;
}

bool getNameSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    nameMutex.lock();
    res.data.push_back(hostname_);
    nameMutex.unlock();

    //ROS_INFO("(Gobot_status) Get Gobot name: %s",res.data[0].c_str());
    return true;
}


bool setPathSrvCallback(gobot_msg_srv::SetPath::Request &req, gobot_msg_srv::SetPath::Response &res){
    pathMutex.lock();
    path_.clear();
    for(int i=0;i<req.path.size();i++)
        path_.push_back(req.path[i]);
    pathMutex.unlock();

    std::ofstream ofsPath(pathFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsPath){
        std::string path_info;
        for(int i = 0; i < path_.size(); i++){
            ofsPath << path_.at(i) << "\n";
            path_info = path_info+path_.at(i)+", ";
        }
        ROS_INFO("(Gobot_status) Set Gobot path: %s",path_info.c_str());
        ofsPath.close();
    }

    return true;
}

bool getPathSrvCallback(gobot_msg_srv::GetPath::Request &req, gobot_msg_srv::GetPath::Response &res){
    pathMutex.lock();
    for(int i=0;i<path_.size();i++)
        res.path.push_back(path_.at(i));
    pathMutex.unlock();
    return true;
}



bool setMuteSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    muteMutex.lock();
    mute_ = req.data[0];
    muteMutex.unlock();

    std::ofstream ofsMute(muteFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsMute){
        ofsMute << mute_;
        ofsMute.close();
    ROS_INFO("(Gobot_status) Set Gobot mute: %d",mute_);
    }

    ros::service::call("/gobot_software/update_status",empty_srv);
    return true;
}

bool getMuteSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    muteMutex.lock();
    res.data.push_back(mute_);
    muteMutex.unlock();
    return true;
}


bool setLoopSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    loopMutex.lock();
    loop_ = req.data[0];
    loopMutex.unlock();

    std::ofstream ofsLoop(pathLoopFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsLoop){
        ofsLoop << loop_;
        ofsLoop.close();
        ROS_INFO("(Gobot_status) Set Gobot loop: %d",loop_);
    }

    return true;
}

bool getLoopSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    loopMutex.lock();
    res.data.push_back(loop_);
    loopMutex.unlock();
    return true;
}


bool setStageSrvCallback(gobot_msg_srv::SetStage::Request &req, gobot_msg_srv::SetStage::Response &res){
    stageMutex.lock();
    stage_ = req.stage;
    stageMutex.unlock();

    std::ofstream ofsStage(pathStageFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsStage){
        ofsStage << stage_;
        ofsStage.close();
        ROS_INFO("(Gobot_status) Set Gobot stage: %d",stage_);
    }

    ros::service::call("/gobot_software/update_status",empty_srv);

    return true;
}

bool getStageSrvCallback(gobot_msg_srv::GetStage::Request &req, gobot_msg_srv::GetStage::Response &res){
    stageMutex.lock();
    res.stage = stage_;
    stageMutex.unlock();

    //ROS_INFO("(Gobot_status) Get Gobot stage: %d",stage_);
    return true;
}


bool setDockStatusSrvCallback(gobot_msg_srv::SetDockStatus::Request &req, gobot_msg_srv::SetDockStatus::Response &res){
    dockStatusMutex.lock();
    dock_status_ = req.status;
    dockStatusMutex.unlock();
    ROS_INFO("(Gobot_status) Set Dock status: %d", dock_status_);
    ros::service::call("/gobot_software/update_status",empty_srv);

    if(dock_status_ == 1 && (gobot_status_!=5 || gobot_text_=="WAITING")){
        std::thread([](){
            ros::service::call("/gobot_recovery/initialize_home",empty_srv);
        }).detach();
    }
    
    return true;
}

bool getDockStatusSrvCallback(gobot_msg_srv::GetDockStatus::Request &req, gobot_msg_srv::GetDockStatus::Response &res){
    dockStatusMutex.lock();
    res.status = dock_status_;
    dockStatusMutex.unlock();
    //ROS_INFO("(Gobot_status) Get Dock status: %d", dock_status_);

    return true;
}


bool setGobotStatusSrvCallback(gobot_msg_srv::SetGobotStatus::Request &req, gobot_msg_srv::SetGobotStatus::Response &res){
    gobotStatusMutex.lock();
    gobot_status_ = req.status;
    gobot_text_ = req.text;
    gobotStatusMutex.unlock();
    ROS_INFO("(Gobot_status) Set Gobot status: %d,%s",gobot_status_,gobot_text_.c_str());
    return true;
}

bool getGobotStatusSrvCallback(gobot_msg_srv::GetGobotStatus::Request &req, gobot_msg_srv::GetGobotStatus::Response &res){
    gobotStatusMutex.lock();
    res.status = gobot_status_;
    res.text = gobot_text_;
    gobotStatusMutex.unlock();
    return true;
}


void initialData(){
    ros::NodeHandle n;
    //read stage
    n.getParam("path_stage_file", pathStageFile);
    std::ifstream ifsStage(pathStageFile, std::ifstream::in);
    if(ifsStage){
        ifsStage >> stage_;
        ifsStage.close();
        ROS_INFO("(Gobot_status) Read Gobot path stage: %d",stage_);
    }
    
    if(stage_==-99 || stage_==-100){
        stage_=0;
        std::ofstream ofsStage(pathStageFile, std::ofstream::out | std::ofstream::trunc);
        if(ofsStage){
            ofsStage << stage_;
            ofsStage.close();
            ROS_INFO("(Gobot_status) Set Gobot stage: %d",stage_);
        }
    }    

    //read mute
    n.getParam("mute_file", muteFile);
    std::ifstream ifsMute(muteFile, std::ifstream::in);
    if(ifsMute){
        ifsMute >> mute_;
        ifsMute.close();
        ROS_INFO("(Gobot_status) Read Gobot mute: %d",mute_);
    } 

    //read loop
    n.getParam("path_loop_file", pathLoopFile);
    std::ifstream ifsLoop(pathLoopFile, std::ifstream::in);
    if(ifsLoop){
        ifsLoop >> loop_;
        ifsLoop.close();
        ROS_INFO("(Gobot_status) Read Gobot path loop: %d",loop_);
    } 

    //read path
    n.getParam("path_file", pathFile);
    std::ifstream ifPath(pathFile, std::ifstream::in);
    if(ifPath){
        std::string line;
        while(getline(ifPath, line))
            path_.push_back(line); 
        ifPath.close();
        std::string path_info;
        for(int i = 0; i < path_.size(); i++)
            path_info = path_info+path_.at(i)+", ";
        ROS_INFO("(Gobot_status) Read Gobot path: %s",path_info.c_str());
    }

    //read name
    n.getParam("robot_name_file", nameFile);
    std::ifstream ifsName(nameFile, std::ifstream::in);
    if(ifsName){
        ifsName >> hostname_;
        ifsName.close();
        ROS_INFO("(Gobot_status) Read Gobot name: %s",hostname_.c_str());
    }

    //read home
    n.getParam("home_file", homeFile);
    std::ifstream ifsHome(homeFile, std::ifstream::in);
    if(ifsHome){
        ifsHome >> home_.at(0) >> home_.at(1) >> home_.at(2) >> home_.at(3) >> home_.at(4) >> home_.at(5);
        ifsHome.close();
        ROS_INFO("(Gobot_status) Read Gobot home: [%.2f, %.2f] [%.2f, %.2f, %.2f, %.2f]", std::stod(home_.at(0)),std::stod(home_.at(1)),std::stod(home_.at(2)),std::stod(home_.at(3)),std::stod(home_.at(4)),std::stod(home_.at(5)));
    }

    //read low battery
    n.getParam("low_battery_file", lowBatteryFile);
    std::ifstream ifsBattery(lowBatteryFile, std::ifstream::in);
    if(ifsBattery){
        ifsBattery >> low_battery_;
        ifsBattery.close();
        ROS_INFO("(Gobot_status) Read Gobot battery: %.2f", std::stod(low_battery_));
    }

    //read wifi
    n.getParam("wifi_file", wifiFile);
    std::ifstream ifWifi(wifiFile, std::ifstream::in);
    if(ifWifi){
        std::string line;
        while(getline(ifWifi, line))
            wifi_.push_back(line);
            
        ifWifi.close();
    }
    if(wifi_.size()<1){
        wifi_.push_back("");
        wifi_.push_back("");
    }
    else if(wifi_.size()<2){
        wifi_.push_back("");
    }
    ROS_INFO("(Gobot_status) Read Gobot wifi: name:%s, password:%s",wifi_.at(0).c_str(),wifi_.at(1).c_str());

    n.getParam("deleteWIFI", deleteWifi);

    //record disconnect times
    n.getParam("disconnected_file", disconnectedFile);
    std::ofstream ofsDisconnected(disconnectedFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsDisconnected){
        ofsDisconnected << disconnected;
    }
}

int main(int argc, char* argv[]){

    try{
        ros::init(argc, argv, "gobot_status");
        ros::NodeHandle n;

        initialData();

        ros::ServiceServer setGobotStatusSrv = n.advertiseService("/gobot_status/get_gobot_status", getGobotStatusSrvCallback);
        ros::ServiceServer getGobotStatusSrv = n.advertiseService("/gobot_status/set_gobot_status", setGobotStatusSrvCallback);

        ros::ServiceServer setDockStatusSrv = n.advertiseService("/gobot_status/set_dock_status", setDockStatusSrvCallback);
        ros::ServiceServer getDockStatusSrv = n.advertiseService("/gobot_status/get_dock_status", getDockStatusSrvCallback);

        ros::ServiceServer setStageSrv = n.advertiseService("/gobot_status/set_stage", setStageSrvCallback);
        ros::ServiceServer getStageSrv = n.advertiseService("/gobot_status/get_stage", getStageSrvCallback);

        ros::ServiceServer setPathSrv = n.advertiseService("/gobot_status/set_path", setPathSrvCallback);
        ros::ServiceServer getPathSrv = n.advertiseService("/gobot_status/get_path", getPathSrvCallback);

        ros::ServiceServer setLoopSrv = n.advertiseService("/gobot_status/set_loop", setLoopSrvCallback);
        ros::ServiceServer getLoopSrv = n.advertiseService("/gobot_status/get_loop", getLoopSrvCallback);

        ros::ServiceServer setNameSrv = n.advertiseService("/gobot_status/set_name", setNameSrvCallback);
        ros::ServiceServer getNameSrv = n.advertiseService("/gobot_status/get_name", getNameSrvCallback);

        ros::ServiceServer setHomeSrv = n.advertiseService("/gobot_status/set_home", setHomeSrvCallback);
        ros::ServiceServer getHomeSrv = n.advertiseService("/gobot_status/get_home", getHomeSrvCallback);

        ros::ServiceServer setBatterySrv = n.advertiseService("/gobot_status/set_battery", setBatterySrvCallback);
        ros::ServiceServer getBatterySrv = n.advertiseService("/gobot_status/get_battery", getBatterySrvCallback);

        ros::ServiceServer setMuteSrv = n.advertiseService("/gobot_status/set_mute", setMuteSrvCallback);
        ros::ServiceServer getMuteSrv = n.advertiseService("/gobot_status/get_mute", getMuteSrvCallback);

        ros::ServiceServer setWifiSrv = n.advertiseService("/gobot_status/set_wifi", setWifiSrvCallback);
        ros::ServiceServer getWifiSrv = n.advertiseService("/gobot_status/get_wifi", getWifiSrvCallback);

        ros::ServiceServer disconnectedSrv = n.advertiseService("/gobot_test/disconnected", disconnectedSrvCallback);


        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("(Gobot_status system) Exception : %s", e.what());
    }

    return 0;
}
