#include <gobot_software/gobot_status.hpp>

/*
GOBOT STATUS
25 EXPLORING
21 STOP_EXPLORING/COMPLETE_EXPLORING
20 EXPLORATION
15 DOCKING
11 STOP_DOCKING/FAIL_DOKCING
5  PLAY_PATH/WAITING
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

std::mutex gobotStatusMutex,dockStatusMutex,stageMutex,pathMutex,nameMutex,homeMutex,loopMutex;
std_srvs::Empty empty_srv;

int gobot_status_=-99;
std::string gobot_text_ = "FREE";

//3->go to docking 1->charging 0->not charging -1->failed to docking
int dock_status_ = 0;

std::string pathStageFile;
int stage_ = 0;

std::string pathLoopFile;
int loop_ = 0;

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
        for(int i = 0; i < path_.size(); i++){
            ofsPath << path_.at(i) << "\n";
            ROS_INFO("(Gobot_status) Set Gobot path %d: %s",i,path_.at(i).c_str());
        }
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



bool setLoopSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    loopMutex.lock();
    loop_ = req.data[0];
    loopMutex.unlock();

    std::ofstream ofsLoop(pathLoopFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsLoop){
        ofsLoop << loop_;
        ofsLoop.close();
    }

    ROS_INFO("(Gobot_status) Set Gobot loop: %d",loop_);
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
    }

    ROS_INFO("(Gobot_status) Set Gobot stage: %d",stage_);
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
    if(n.hasParam("path_stage_file")){
        n.getParam("path_stage_file", pathStageFile);

        std::ifstream ifsStage(pathStageFile, std::ifstream::in);
        if(ifsStage){
            ifsStage >> stage_;
            ifsStage.close();
            ROS_INFO("(Gobot_status) Read Gobot path stage: %d",stage_);
        } 
    }   

    //read loop
    if(n.hasParam("path_loop_file")){
        n.getParam("path_loop_file", pathLoopFile);

        std::ifstream ifsLoop(pathLoopFile, std::ifstream::in);
        if(ifsLoop){
            ifsLoop >> loop_;
            ifsLoop.close();
            ROS_INFO("(Gobot_status) Read Gobot path loop: %d",loop_);
        } 
    } 

    //read path
    if(n.hasParam("path_file")){
        n.getParam("path_file", pathFile);

        std::ifstream ifsfile(pathFile, std::ifstream::in);
        if(ifsfile){
            std::string line;
            while(getline(ifsfile, line))
                path_.push_back(line);
                
            ifsfile.close();
            for(int i = 0; i < path_.size(); i++)
                ROS_INFO("(Gobot_status) Read Gobot path %d: %s",i,path_.at(i).c_str());
        }
    }

    //read name
    if(n.hasParam("robot_name_file")){
        n.getParam("robot_name_file", nameFile);

        std::ifstream ifsName(nameFile, std::ifstream::in);
        if(ifsName){
            ifsName >> hostname_;
            ifsName.close();
            ROS_INFO("(Gobot_status) Read Gobot name: %s",hostname_.c_str());
        }
    }

    //read home
    if(n.hasParam("home_file")){
        n.getParam("home_file", homeFile);

        std::ifstream ifsHome(homeFile, std::ifstream::in);
        if(ifsHome){
            ifsHome >> home_.at(0) >> home_.at(1) >> home_.at(2) >> home_.at(3) >> home_.at(4) >> home_.at(5);
            ifsHome.close();
            ROS_INFO("(Gobot_status) Read Gobot home: [%.2f, %.2f] [%.2f, %.2f, %.2f, %.2f]", std::stod(home_.at(0)),std::stod(home_.at(1)),std::stod(home_.at(2)),std::stod(home_.at(3)),std::stod(home_.at(4)),std::stod(home_.at(5)));
        }
    }

    if(n.hasParam("disconnected_file")){
        n.getParam("disconnected_file", disconnectedFile);

        std::ofstream ofsDisconnected(disconnectedFile, std::ofstream::out | std::ofstream::trunc);
        if(ofsDisconnected){
            ofsDisconnected << disconnected;
        }
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

        ros::ServiceServer disconnectedSrv = n.advertiseService("/gobot_test/disconnected", disconnectedSrvCallback);


        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("(Gobot_status system) Exception : %s", e.what());
    }

    return 0;
}
