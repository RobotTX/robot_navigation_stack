#include <gobot_software/gobot_status.hpp>
#include <std_msgs/String.h>
/*
GOBOT STATUS
25 EXPLORING
21 STOP_EXPLORING/COMPLETE_EXPLORING
20 EXPLORATION
15 DOCKING
11 STOP_DOCKING/FAIL_DOKCING/COMPLETE_DOCKING
5  PLAY_PATH/WAITING/DELAY
4  PAUSE_PATH
1  STOP_PATH
0  COMPLETE_PATH/ABORTED_PATH/COMPLETE_POINT
-1 ROBOT_READY

DOCK STATUS
3  GO TO CHARGING
1  CHARING
0  NOT CHARGING
-1 FAILED TO GO TO CHARGING
*/

ros::Publisher disco_pub,initial_pose_publisher, mute_pub, update_info_pub, status_pub;

static const std::string sep = std::string(1, 31);

std::mutex gobotStatusMutex,dockStatusMutex,stageMutex,pathMutex,nameMutex,homeMutex,loopMutex,muteMutex,wifiMutex,batteryMutex,speedMutex;
std_srvs::Empty empty_srv;

std::string wifiFile, deleteWifi;
std::vector<std::string> wifi_;

gobot_msg_srv::SetStringArray update_path;

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

std::string speedFile;
std::string linear_spd_="0.4";
std::string angular_spd_="40";

std::string pathFile;
std::vector<std::string> path_;

std::string nameFile;
std::string hostname_="Go_Gobot";

std::string homeFile;
std::vector<std::string> home_(6,"0");

std::string disconnectedFile;
int disconnected = 0;

bool charging_ = false;
int battery_percent_ = 51;

std::string recordBatteryFile;
int charging_current_ = 0;
ros::Time record_time_;
int record_battery_ = 0;

bool collision = false;

robot_class::SetRobot SetRobot;


std::string getCurrentTime(){
  std::time_t now = std::time(0);
  std::stringstream transTime;
  transTime << std::put_time(localtime(&now), "%F-%H:%M:%S");
  return transTime.str();
}

//updated information for ping_server_new
void updateStatus(){
    bool muteFlag = (mute_) ? 1 : 0;
    std_msgs::String update_status;
    update_status.data = hostname_ + sep + std::to_string(stage_) + sep + std::to_string(battery_percent_) + 
                                    sep + std::to_string(muteFlag) + sep + std::to_string(dock_status_);
    update_info_pub.publish(update_status);
}

//change robot led and sound to inform people its status
void robotResponse(int status, std::string text){
    //permanent green case
    if (text=="COMPLETE_EXPLORING" || text=="COMPLETE_PATH" || text=="COMPLETE_POINT") {
        SetRobot.setLed(0,{"green"});
        SetRobot.setSound(1,2);
    }
    else if(text=="STOP_EXPLORING" || text=="STOP_DOCKING" || text=="STOP_PATH" || text=="PAUSE_PATH"){
        SetRobot.setLed(0,{"blue"});
    }
    else if(text=="FAIL_DOCKING" || text=="ABORTED_PATH"){
        SetRobot.setLed(0,{"red"});
        SetRobot.setSound(3,2);
    }
    else if(text=="PLAY_PATH" || text=="PLAY_POINT" || text=="EXPLORING"){
        SetRobot.setLed(1,{"green","white"});
    }
    else if(text=="DELAY" || text=="WAITING"){
        SetRobot.setLed(1,{"blue","white"});
    }
    else if(text=="DOCKING"){
        SetRobot.setLed(1,{"yellow","cyan"});
    }
    else if(text=="COMPLETE_DOCKING"){
        SetRobot.setSound(1,2);
    }
}

//publish pose
void publishInitialpose(geometry_msgs::PoseWithCovarianceStamped pose){
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    //x-xy-y,yaw-yaw
    pose.pose.covariance[0] = 0.01;
    pose.pose.covariance[7] = 0.01;
    pose.pose.covariance[35] = 0.01;
    if(pose.pose.pose.position.x == 0 && pose.pose.pose.position.y == 0 && pose.pose.pose.orientation.w == 0){
        ROS_ERROR("(gobot_status) Robot probably got no home, set it position to map origin");
        pose.pose.pose.orientation.w = 1.0;
    }
    initial_pose_publisher.publish(pose);
}

void setHomePose(){
    geometry_msgs::PoseWithCovarianceStamped home_pose;
    home_pose.pose.pose.position.x = std::stod(home_.at(0));
    home_pose.pose.pose.position.y = std::stod(home_.at(1));
    home_pose.pose.pose.orientation.z = std::stod(home_.at(2));
    home_pose.pose.pose.orientation.z = std::stod(home_.at(3));
    home_pose.pose.pose.orientation.z = std::stod(home_.at(4));
    home_pose.pose.pose.orientation.w = std::stod(home_.at(5));
    publishInitialpose(home_pose);
    ROS_INFO("(Gobot Status) Robot is charging, set its pose to be home pose");
}

//initialize robot to be home if charging
bool initializeHomeSrcCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(charging_){
        setHomePose();
        SetRobot.setSound(1,2);
    }
    return true;
}


bool getUpdateStatusSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    bool muteFlag = (mute_) ? 1 : 0;
    res.data = hostname_ + sep + std::to_string(stage_) + sep + std::to_string(battery_percent_) + 
               sep + std::to_string(muteFlag) + sep + std::to_string(dock_status_);
    return true;
}

bool setWifiSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){    
    if(req.data.size()!=2){
        ROS_ERROR("(Gobot_status) Receive wrong wifi information");
        return false;
    }
    if(wifi_.at(0)==req.data[0] && wifi_.at(1)==req.data[1]){
        ROS_INFO("(Gobot_status) Receive same wifi info: name:%s, password:%s",wifi_.at(0).c_str(),wifi_.at(1).c_str());
        return false;
    }
    //disconnect all server
    ros::service::call("/gobot_software/disconnet_servers",empty_srv);

    //if previous wifi is not empty and receive a new wifi, we delete the previous wifi in the netowrk list
    //delete previously connected wifi
    const std::string deleteWIFI_script = "sudo sh " + deleteWifi + " " + wifiFile;
    system(deleteWIFI_script.c_str());
    wifiMutex.lock();
    wifi_.clear();
    for(int i=0;i<req.data.size();i++)
        wifi_.push_back(req.data[i]);

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

bool getWifiSrvCallback(gobot_msg_srv::GetStringArray::Request &req, gobot_msg_srv::GetStringArray::Response &res){
    wifiMutex.lock();
    for(int i=0;i<wifi_.size();i++)
        res.data.push_back(wifi_.at(i));
    wifiMutex.unlock();
    return true;
}


bool setHomeSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    if(req.data.size()!=6){
        ROS_ERROR("(Gobot_status) Receive wrong home information");
        return false;
    }
    
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

bool getHomeSrvCallback(gobot_msg_srv::GetStringArray::Request &req, gobot_msg_srv::GetStringArray::Response &res){
    homeMutex.lock();
    for(int i=0;i<home_.size();i++)
        res.data.push_back(home_.at(i));
    homeMutex.unlock();
    return true;
}


bool setBatterySrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
    batteryMutex.lock();
    low_battery_=req.data;
    batteryMutex.unlock();

    std::ofstream ofsBattery(lowBatteryFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsBattery){
        ofsBattery << low_battery_;
        ofsBattery.close();
        ROS_INFO("(Gobot_status) set Gobot battery level: %.2f", std::stod(low_battery_));
    }

    return true;
}

bool getBatterySrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    batteryMutex.lock();
    res.data = low_battery_;
    batteryMutex.unlock();
    return true;
}

bool setSpeedSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    if(req.data.size()!=2){
        ROS_ERROR("(Gobot_status) Receive wrong set_speed information");
        return false;
    }
    
    speedMutex.lock();
    linear_spd_=req.data[0];
    angular_spd_=req.data[1];
    speedMutex.unlock();

    std::ofstream ofSpeed(speedFile, std::ofstream::out | std::ofstream::trunc);
    if(ofSpeed){
        ofSpeed << linear_spd_ << " " << angular_spd_;
        ofSpeed.close();
        ROS_INFO("(Gobot_status) set Gobot speed: %.2f, %.2f", std::stod(linear_spd_),std::stod(angular_spd_));
    }

    dynamic_reconfigure::Reconfigure config;
    dynamic_reconfigure::DoubleParameter linear_param,angular_param;
    linear_param.name = "max_vel_x";
    linear_param.value = std::stod(linear_spd_);
    angular_param.name = "max_vel_theta";
    angular_param.value = std::stod(angular_spd_)*3.14159/180;

    config.request.config.doubles.push_back(linear_param);
    config.request.config.doubles.push_back(angular_param);
    if(ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config)){
        return true;
    }

    return true;
}

bool getSpeedSrvCallback(gobot_msg_srv::GetStringArray::Request &req, gobot_msg_srv::GetStringArray::Response &res){
    speedMutex.lock();
    res.data.push_back(linear_spd_);
    res.data.push_back(angular_spd_);
    speedMutex.unlock();
    return true;
}

bool setNameSrvCallback(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
    nameMutex.lock();
    hostname_=req.data;
    nameMutex.unlock();

    std::ofstream ofsName(nameFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsName){
        ofsName << hostname_;
        ofsName.close();
        ROS_INFO("(Gobot_status) Set Gobot name: %s",hostname_.c_str());
    }

    updateStatus();

    return true;
}

bool getNameSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    nameMutex.lock();
    res.data = hostname_;
    nameMutex.unlock();

    //ROS_INFO("(Gobot_status) Get Gobot name: %s",res.data.c_str());
    return true;
}


bool setPathSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    pathMutex.lock();
    path_.clear();
    for(int i=0;i<req.data.size();i++)
        path_.push_back(req.data[i]);
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
    
    update_path.request.data = req.data;
    std::thread([](){
        ros::service::call("/gobot_function/update_path", update_path);
    }).detach();
    return true;
}

bool getPathSrvCallback(gobot_msg_srv::GetStringArray::Request &req, gobot_msg_srv::GetStringArray::Response &res){
    pathMutex.lock();
    for(int i=0;i<path_.size();i++)
        res.data.push_back(path_.at(i));
    pathMutex.unlock();
    return true;
}



bool setMuteSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    muteMutex.lock();
    mute_ = req.data;

    std_msgs::Int8 data;
    data.data = mute_;
    mute_pub.publish(data);
    muteMutex.unlock();

    std::ofstream ofsMute(muteFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsMute){
        ofsMute << mute_;
        ofsMute.close();
    ROS_INFO("(Gobot_status) Set Gobot mute: %d",mute_);
    }
    updateStatus();
    
    return true;
}

bool getMuteSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    muteMutex.lock();
    res.data = mute_;
    muteMutex.unlock();
    return true;
}


bool setLoopSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    loopMutex.lock();
    loop_ = req.data;
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
    res.data = loop_;
    loopMutex.unlock();
    return true;
}


bool setStageSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    stageMutex.lock();
    stage_ = req.data;
    stageMutex.unlock();

    std::ofstream ofsStage(pathStageFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsStage){
        ofsStage << stage_;
        ofsStage.close();
        ROS_INFO("(Gobot_status) Set Gobot stage: %d",stage_);
    }
    
    updateStatus();

    return true;
}

bool getStageSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    stageMutex.lock();
    res.data = stage_;
    stageMutex.unlock();

    //ROS_INFO("(Gobot_status) Get Gobot stage: %d",stage_);
    return true;
}


bool setDockStatusSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    dockStatusMutex.lock();
    dock_status_ = req.data;
    dockStatusMutex.unlock();
    ROS_INFO("(Gobot_status) Set Dock status: %d", dock_status_);
    
    charging_ = dock_status_==1 ? true : false;
    
    updateStatus();

    if(dock_status_==1 && ros::service::exists("/gobot_startup/pose_ready",false)){
        if(gobot_status_!=5 || gobot_text_=="WAITING"){
            setHomePose();
        }
    }
    
    return true;
}

bool getDockStatusSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    dockStatusMutex.lock();
    res.data = dock_status_;
    dockStatusMutex.unlock();
    //ROS_INFO("(Gobot_status) Get Dock status: %d", dock_status_);

    return true;
}


bool setGobotStatusSrvCallback(gobot_msg_srv::SetGobotStatus::Request &req, gobot_msg_srv::SetGobotStatus::Response &res){
    gobotStatusMutex.lock();
    gobot_status_ = req.status;
    gobot_text_ = req.text;
    std_msgs::Int8 data;
    data.data = gobot_status_;
    status_pub.publish(data);
    robotResponse(gobot_status_,gobot_text_);

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

bool isChargingService(gobot_msg_srv::IsCharging::Request &req, gobot_msg_srv::IsCharging::Response &res){
    res.isCharging = charging_;
    return true;
}

bool PercentService(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    res.data = battery_percent_;
    return true;
}

bool disconnectedSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    /*
    disconnected++;
    std::ofstream ofsDisconnected(disconnectedFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsDisconnected)
        ofsDisconnected << disconnected;
    */

    //test purpose
    ros::service::call("/gobot_software/disconnet_servers",empty_srv);
    return true;
}

bool recordBatterySrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    record_battery_ = req.data;
    if (record_battery_==-1){
        std::ofstream ofsStage(recordBatteryFile, std::ofstream::out | std::ofstream::trunc);
        if(ofsStage){
            ofsStage.close();
        }
    }
    return true;
}

void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg){
    battery_percent_ = msg->BatteryStatus;
    charging_current_ = msg->ChargingCurrent;
    //for recording test purpose
    if(record_battery_==1 && (ros::Time::now() - record_time_).toSec() >= 5.0){
        std::string record_date = getCurrentTime();
        std::ofstream ofsStage(recordBatteryFile, std::ofstream::out | std::ofstream::app);
        if(ofsStage){
            ofsStage << record_date << "  " <<charging_current_<<std::endl;
            ofsStage.close();
            ROS_INFO("(Gobot_status) Recorded battery data: %s  %d",record_date.c_str(),charging_current_);
            record_time_ = ros::Time::now();
        }
    }
}

//for changing led/sound purpose. There should be no change when docking
void bumpersCallback(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    if(gobot_status_!=15){
        bool front = !(bumpers->bumper1 && bumpers->bumper2 && bumpers->bumper3 && bumpers->bumper4);
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        if(front || back){
            if(!collision){
                collision = true;
                SetRobot.setLed(1,{"red","white"});
                SetRobot.setSound(3,1);
            }
        }
        else if(collision){
            collision = false;
            SetRobot.setSound(1,1);
        }
    }
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
    
    if(stage_<0){
        if(stage_==-99 || stage_==-100){
            stage_=0;
        }
        else{
            stage_=-stage_-1;
        }
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
        std_msgs::Int8 data;
        data.data = mute_;
        mute_pub.publish(data);
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

    //read speed
    n.getParam("speed_file", speedFile);
    std::ifstream ifsSpeed(speedFile, std::ifstream::in);
    if(ifsSpeed){
        ifsSpeed >> linear_spd_ >> angular_spd_;
        ifsSpeed.close();
        ROS_INFO("(Gobot_status) Read Gobot speed: %.2f, %.2f", std::stod(linear_spd_),std::stod(angular_spd_));
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
        ofsDisconnected.close();
    }

    n.getParam("record_battery_file", recordBatteryFile);
}

int main(int argc, char* argv[]){

    try{
        ros::init(argc, argv, "gobot_status");
        ros::NodeHandle n;

        SetRobot.initialize();
        mute_pub = n.advertise<std_msgs::Int8>("/gobot_status/mute", 1, true);
        update_info_pub = n.advertise<std_msgs::String>("/gobot_status/update_information", 1, true);
        status_pub = n.advertise<std_msgs::Int8>("/gobot_status/gobot_status", 1, true);

        initialData();

        initial_pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

        ros::ServiceServer initializeHome = n.advertiseService("/gobot_recovery/initialize_home",initializeHomeSrcCallback);

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

        ros::ServiceServer setSpeedSrv = n.advertiseService("/gobot_status/set_speed", setSpeedSrvCallback);
        ros::ServiceServer getSpeedSrv = n.advertiseService("/gobot_status/get_speed", getSpeedSrvCallback);

        ros::ServiceServer setMuteSrv = n.advertiseService("/gobot_status/set_mute", setMuteSrvCallback);
        ros::ServiceServer getMuteSrv = n.advertiseService("/gobot_status/get_mute", getMuteSrvCallback);

        ros::ServiceServer setWifiSrv = n.advertiseService("/gobot_status/set_wifi", setWifiSrvCallback);
        ros::ServiceServer getWifiSrv = n.advertiseService("/gobot_status/get_wifi", getWifiSrvCallback);

        ros::ServiceServer isChargingSrv = n.advertiseService("/gobot_status/charging_status", isChargingService);
        ros::ServiceServer PercentSrv = n.advertiseService("/gobot_status/battery_percent", PercentService);

        ros::ServiceServer getUpdateStatusSrv = n.advertiseService("/gobot_status/get_update_status", getUpdateStatusSrvCallback);

        ros::ServiceServer disconnectedSrv = n.advertiseService("/gobot_test/disconnected", disconnectedSrvCallback);
        ros::ServiceServer recordBatterySrv = n.advertiseService("/gobot_test/record_battery", recordBatterySrvCallback);

        ros::Subscriber battery = n.subscribe("/gobot_base/battery_topic",1, batteryCallback);
        ros::Subscriber bumpers = n.subscribe("/gobot_base/bumpers_topic",1, bumpersCallback);

        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("(Gobot_status system) Exception : %s", e.what());
    }

    return 0;
}
