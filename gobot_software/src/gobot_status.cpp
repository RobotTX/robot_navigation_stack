#include <gobot_software/gobot_status.hpp>
/*
========================
GOBOT STATUS
25 EXPLORING
22 START_EXPLORING
21 STOP_EXPLORING/COMPLETE_EXPLORING
20 EXPLORATION
15 DOCKING
11 STOP_DOCKING/FAIL_DOKCING/COMPLETE_DOCKING
16 TRACKING
12 STOP_TRACKING/FAIL_TRACKING
5  PLAY_PATH/PLAY_POINT/WAITING/DELAY/AUDIO_DELAY
4  PAUSE_PATH
1  STOP_PATH
0  COMPLETE_PATH/ABORTED_PATH/COMPLETE_POINT
-1 ROBOT_READY
-2 STARTUP_READY
-3 SCAN_READY
-4 MAP_READY
===================================
DOCK STATUS
3  GO TO CHARGING
1  CHARING
0  NOT CHARGING
-1 FAILED TO GO TO CHARGING
-3 BATTERY STATUS NOT CLEAR
*/

ros::Publisher volume_pub, update_info_pub, status_pub;

static const std::string sep = std::string(1, 31);

std::mutex gobotStatusMutex,dockStatusMutex,stageMutex,pathMutex,nameMutex,homeMutex,loopMutex,volumeMutex,wifiMutex,batteryMutex,speedMutex;
std_srvs::Empty empty_srv;

std::string mission_failed_mp3, auto_docking_mp3, docking_complete_mp3, auto_scan_mp3, scan_complete_mp3, startup_mp3, scan_mp3, 
            reload_map_mp3, mission_complete_mp3, abort_navigation_mp3;

std::string wifiFile, deleteWifi;
std::vector<std::string> wifi_;

gobot_msg_srv::SetStringArray update_path;

std::string volumeFile;
int volume_ = 70;

int gobot_status_=-99;
std::string gobot_text_ = "FREE";

//3->go to docking 1->charging 0->not charging -1->failed to docking
int dock_status_ = 0;

std::string pathStageFile;
int stage_ = 0;

std::string pathLoopFile;
int loop_ = 0;

std::string lowBatteryFile;
std::string low_battery_="0";

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

bool charging_status_ = false, battery_charging_ = false;
int battery_percent_ = 51;
ros::Time battery_time_;

std::string recordBatteryFile;
int charging_current_ = 0;
ros::Time record_time_;
int record_battery_ = 0;

bool collision = false;

robot_class::SetRobot SetRobot;

//0-auto, 1-manual
int robot_mode_ = 0;

//0-detached   1-attached
int magnet_status_ = 0;

//****************************** CALLBACK ******************************
void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg){
    battery_percent_ = msg->BatteryStatus;
    charging_current_ = msg->ChargingCurrent;
    battery_charging_ = msg->ChargingFlag;

    //in case robot move away from CS, but battery charging status is not changed, so reset battery charging status after 60s
    if(battery_charging_ && !charging_status_){
        if(ros::Time::now()-battery_time_ > ros::Duration(60.0)){
            gobot_msg_srv::SetBool charging;
            charging.request.data = false;
            ros::service::call("/gobot_base/set_charging",charging);
        }
    }
    /*//for recording test purpose
    if(record_battery_==1 && (ros::Time::now() - record_time_) >= ros::Duration(5.0)){
        std::string record_date = getCurrentTime();
        std::ofstream ofsStage(recordBatteryFile, std::ofstream::out | std::ofstream::app);
        if(ofsStage){
            ofsStage << record_date << "  " <<charging_current_<<std::endl;
            ofsStage.close();
            ROS_INFO("(STATUS_SYSTEM) Recorded battery data: %s  %d",record_date.c_str(),charging_current_);
            record_time_ = ros::Time::now();
        }
    }
    */
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

void magnetCb(const std_msgs::Int8::ConstPtr& msg){
    magnet_status_ = msg->data;
}

//****************************** SERVICE ******************************
//initialize robot to be home if charging
bool initializeHomeSrcCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(charging_status_){
        setHomePose();
        SetRobot.setSound(1,2);
    }
    return true;
}

bool updateStatusSrvCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    updateStatus();
    return true;
}

bool getUpdateStatusSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    res.data = getUpdateStatus();
    return true;
}

bool setWifiSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){    
    if(req.data.size()!=2){
        ROS_ERROR("(STATUS_SYSTEM) Receive wrong wifi information");
        return false;
    }
    if(wifi_.at(0)==req.data[0] && wifi_.at(1)==req.data[1]){
        ROS_INFO("(STATUS_SYSTEM) Receive same wifi info: name:%s, password:%s",wifi_.at(0).c_str(),wifi_.at(1).c_str());
        return false;
    }
    //disconnect all server
    ros::service::call("/gobot_software/disconnet_servers",empty_srv);
    //stop robot
    std::thread([](){
        ros::service::call("/gobot_command/pause_path", empty_srv);
    }).detach();

    //if previous wifi is not empty and receive a new wifi, we delete the previous wifi in the netowrk list
    //delete previously connected wifi
    std::string oldWifi = wifi_.at(0)=="" ? "null_wifi" : wifi_.at(0);
    std::string newWifi = req.data[0]=="" ? "null_wifi" : req.data[0];
    const std::string deleteWIFI_script = "sudo sh " + deleteWifi + " " + oldWifi + " " + newWifi + " " + req.data[1]  + " &";  //run in thread
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
        ROS_INFO("(STATUS_SYSTEM) Set robot wifi: name:%s, password:%s",wifi_.at(0).c_str(),wifi_.at(1).c_str());
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
        ROS_ERROR("(STATUS_SYSTEM) Receive wrong home information");
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
        ROS_INFO("(STATUS_SYSTEM) Set robot home: [%f, %f] [%f, %f, %f, %f]", std::stod(home_.at(0)),std::stod(home_.at(1)),std::stod(home_.at(2)),std::stod(home_.at(3)),std::stod(home_.at(4)),std::stod(home_.at(5)));
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
        ROS_INFO("(STATUS_SYSTEM) Set robot battery level: %.2f", std::stod(low_battery_));
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
        ROS_ERROR("(STATUS_SYSTEM) Receive wrong set_speed information");
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
        ROS_INFO("(STATUS_SYSTEM) Set robot speed: %.2f, %.2f", std::stod(linear_spd_),std::stod(angular_spd_));
    }

    //set the navigation speed limit
    dynamic_reconfigure::Reconfigure config;
    dynamic_reconfigure::DoubleParameter linear_param,angular_param;
    linear_param.name = "max_vel_x";
    linear_param.value = std::stod(linear_spd_);
    angular_param.name = "max_vel_theta";
    angular_param.value = SetRobot.degreeToRad(std::stod(angular_spd_));

    config.request.config.doubles.push_back(linear_param);
    config.request.config.doubles.push_back(angular_param);
    ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config);

    //update UI manual teleop speed limit
    gobot_msg_srv::SetFloatArray change_speed;
    change_speed.request.data.push_back(linear_param.value);
    change_speed.request.data.push_back(angular_param.value);
    ros::service::call("/gobot_software/change_speed",change_speed);

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
        ROS_INFO("(STATUS_SYSTEM) Set robot name: %s",hostname_.c_str());
    }

    updateStatus();

    return true;
}

bool getNameSrvCallback(gobot_msg_srv::GetString::Request &req, gobot_msg_srv::GetString::Response &res){
    nameMutex.lock();
    res.data = hostname_;
    nameMutex.unlock();

    //ROS_INFO("(STATUS_SYSTEM) Get Gobot name: %s",res.data.c_str());
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
        ROS_INFO("(STATUS_SYSTEM) Set robot path: %s",path_info.c_str());
        ofsPath.close();
    }

    //clear the audio files attached to previous path
    ros::service::call("/gobot_software/clear_audio", empty_srv);

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



bool setVolumeSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    //sound volume range from 40% to 100%
    //system volume fix at 95%
    //volume below 40% is treated as mute
    volumeMutex.lock();
    bool pre_volume = volume_;
    switch (req.data) {
        case 0:
            volume_ = 0;
        break;

        case 1:
            if(volume_ > 97)
                volume_ = volume_-1;
            else
                volume_ = 0;
        break;

        case 10:
            if(volume_==0)
                volume_ = 97;
            else if(volume_ < 100)
                volume_ = volume_+1;
        break;

        case 97:
            volume_ = 97;
        break;

        default:
        break;
    }

    std_msgs::Int8 data;
    data.data = volume_;
    volume_pub.publish(data);
    volumeMutex.unlock();

    SetRobot.changeVolume(volume_);
    //if change from mute to unmute or unmute to mute, update UI side
    if((pre_volume==0&&volume_>0) || (pre_volume>0&&volume_==0))
        updateStatus();

    std::ofstream ofsVolume(volumeFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsVolume){
        ofsVolume << volume_;
        ofsVolume.close();
    ROS_INFO("(STATUS_SYSTEM) Set robot volume: %d",volume_);
    }
    return true;
}

bool getVolumeSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    volumeMutex.lock();
    res.data = volume_;
    volumeMutex.unlock();
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
        ROS_INFO("(STATUS_SYSTEM) Set robot loop: %d",loop_);
    }

    return true;
}

bool getLoopSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    loopMutex.lock();
    res.data = loop_;
    loopMutex.unlock();
    return true;
}


bool setModeSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    robot_mode_ = req.data;
    ROS_INFO("(STATUS_SYSTEM) Set robot mode: %d",robot_mode_);
    updateStatus();

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
        ROS_INFO("(STATUS_SYSTEM) Set robot stage: %d",stage_);
    }
    
    updateStatus();

    return true;
}

bool getStageSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    stageMutex.lock();
    res.data = stage_;
    stageMutex.unlock();

    //ROS_INFO("(STATUS_SYSTEM) Get Gobot stage: %d",stage_);
    return true;
}


bool setDockStatusSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    dockStatusMutex.lock();
    dock_status_ = req.data;
    dockStatusMutex.unlock();
    ROS_INFO("(STATUS_SYSTEM) Set Dock status: %d", dock_status_);
    
    charging_status_ = dock_status_==1 ? true : false;
    if(!charging_status_){
        battery_time_ = ros::Time::now();
    }
    
    updateStatus();

    if(charging_status_ && ros::service::exists("/gobot_startup/pose_ready",false)){
        if(gobot_status_!=5 || gobot_text_=="WAITING"){
            setHomePose();
        }
    }
    
    return true;
}

bool getDockStatusSrvCallback(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    //in case robot move away from CS, but battery charging status is not changed
    //return -3 to indicate this case
    if(battery_charging_ && !charging_status_){
        res.data = -3;
    }
    else{
        dockStatusMutex.lock();
        res.data = dock_status_;
        dockStatusMutex.unlock();
    }
    return true;
}

bool isChargingSrvCallback(gobot_msg_srv::IsCharging::Request &req, gobot_msg_srv::IsCharging::Response &res){
    res.isCharging = charging_status_;
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
    
    ROS_INFO("(STATUS_SYSTEM) Set robot status: %d,%s",gobot_status_,gobot_text_.c_str());
    return true;
}

bool getGobotStatusSrvCallback(gobot_msg_srv::GetGobotStatus::Request &req, gobot_msg_srv::GetGobotStatus::Response &res){
    gobotStatusMutex.lock();
    res.status = gobot_status_;
    res.text = gobot_text_;
    gobotStatusMutex.unlock();
    return true;
}

bool PercentService(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    res.data = battery_percent_;
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

//****************************** FUNCTIONS ******************************
std::string getCurrentTime(){
  std::time_t now = std::time(0);
  std::stringstream transTime;
  transTime << std::put_time(localtime(&now), "%F-%H:%M:%S");
  return transTime.str();
}

//get updated information
// 1-hostname, 2-stage, 3-battery_percent, 4-mute, 5-dock_status, 6-robot_mode, *7-magnet_status(have when magnet_status==1)
std::string getUpdateStatus(){
    bool muteFlag = (volume_==0) ? 1 : 0;
    std::string result = hostname_ + sep + std::to_string(stage_) + sep + std::to_string(battery_percent_) + 
               sep + std::to_string(muteFlag) + sep + std::to_string(dock_status_) + sep + std::to_string(robot_mode_);
        
    //if magnet is attached to sth, update UI side
    if(magnet_status_==1)
        result = result + sep + std::to_string(magnet_status_);
    return result;
} 

//send updated information for ping_servers
void updateStatus(){
    std_msgs::String update_status;
    update_status.data = getUpdateStatus();
    update_info_pub.publish(update_status);
}


void playSystemAudio(std::string file){
    SetRobot.playSystemAudio(file, volume_);
}

//change robot led and sound to inform people its status
void robotResponse(int status, std::string text){
    ros::NodeHandle n;
    //permanent green case
    if (text=="COMPLETE_EXPLORING" || text=="COMPLETE_PATH" || text=="COMPLETE_POINT" || text=="COMPLETE_TRACKING") {
        SetRobot.setLed(0,{"green"});
        SetRobot.setSound(1,2);
        if(text=="COMPLETE_EXPLORING"){
            n.getParam("scan_complete_mp3", scan_complete_mp3);
            std::thread t_audio(playSystemAudio, scan_complete_mp3);
            t_audio.detach();
        }
        else if(text=="COMPLETE_PATH" || text=="COMPLETE_TRACKING"){
            n.getParam("mission_complete_mp3", mission_complete_mp3);
            std::thread t_audio(playSystemAudio, mission_complete_mp3);
            t_audio.detach();
        }
    }
    else if(text=="STOP_PATH" || text=="PAUSE_PATH"){
        SetRobot.killAudio();
        SetRobot.setLed(0,{"blue"});
    }
    else if(text=="PLAY_PATH" || text=="PLAY_POINT" || text=="EXPLORING"){
        SetRobot.setLed(1,{"green","white"});
    }
    else if(text=="DELAY" || text=="WAITING" || text=="AUDIO_DELAY"){
        SetRobot.setLed(1,{"blue","white"});
    }
    else if(text=="FAIL_DOCKING" || text=="ABORTED_PATH" || text=="FAIL_TRACKING"){
        SetRobot.setLed(0,{"red"});
        SetRobot.setSound(3,2);
        if(text=="ABORTED_PATH"){
            n.getParam("abort_navigation_mp3", abort_navigation_mp3);
            std::thread t_audio(playSystemAudio, abort_navigation_mp3);
            t_audio.detach();
        }
        else{
            n.getParam("mission_failed_mp3", mission_failed_mp3);
            std::thread t_audio(playSystemAudio, mission_failed_mp3);
            t_audio.detach();
        }
    }
    else if(text=="DOCKING"){
        SetRobot.setLed(1,{"yellow","cyan"});
        n.getParam("auto_docking_mp3", auto_docking_mp3);
        std::thread t_audio(playSystemAudio, auto_docking_mp3);
        t_audio.detach();
    }
    else if(text=="TRACKING"){
        SetRobot.setLed(1,{"green","blue"});
    }
    else if(text=="STOP_EXPLORING" || text=="STOP_DOCKING" || text=="STOP_TRACKING"){
        SetRobot.setLed(0,{"blue"});
    }
    else if(text=="COMPLETE_DOCKING"){
        SetRobot.setSound(1,2);
        n.getParam("docking_complete_mp3", docking_complete_mp3);
        std::thread t_audio(playSystemAudio, docking_complete_mp3);
        t_audio.detach();
    }
    else if(text=="START_EXPLORING"){
        n.getParam("auto_scan_mp3", auto_scan_mp3);
        std::thread t_audio(playSystemAudio, auto_scan_mp3);
        t_audio.detach();
    }
    else if(text=="STARTUP_READY"){
        n.getParam("startup_mp3", startup_mp3);
        std::thread t_audio(playSystemAudio, startup_mp3);
        t_audio.detach();
    }
    else if(text=="SCAN_READY"){
        n.getParam("scan_mp3", scan_mp3);
        std::thread t_audio(playSystemAudio, scan_mp3);
        t_audio.detach();
    }
    else if(text=="RELOAD_MAP"){
        n.getParam("reload_map_mp3", reload_map_mp3);
        std::thread t_audio(playSystemAudio, reload_map_mp3);
        t_audio.detach();
    }
}

void setHomePose(){
    geometry_msgs::PoseWithCovarianceStamped home_pose;
    home_pose.pose.pose.position.x = std::stod(home_.at(0));
    home_pose.pose.pose.position.y = std::stod(home_.at(1));
    home_pose.pose.pose.orientation.z = std::stod(home_.at(2));
    home_pose.pose.pose.orientation.z = std::stod(home_.at(3));
    home_pose.pose.pose.orientation.z = std::stod(home_.at(4));
    home_pose.pose.pose.orientation.w = std::stod(home_.at(5));
    SetRobot.setInitialpose(home_pose);
    ROS_INFO("(STATUS_SYSTEM) Robot is charging, set its pose to be home pose");
}

void initialData(){
    ros::NodeHandle n;

    //read stage
    n.getParam("path_stage_file", pathStageFile);
    std::ifstream ifsStage(pathStageFile, std::ifstream::in);
    if(ifsStage){
        ifsStage >> stage_;
        ifsStage.close();
        ROS_INFO("(STATUS_SYSTEM) Read robot path stage: %d",stage_);
    }
    
    if(stage_<0){
        stage_ = -stage_ - 1;
        std::ofstream ofsStage(pathStageFile, std::ofstream::out | std::ofstream::trunc);
            if(ofsStage){
                ofsStage << stage_;
                ofsStage.close();
                ROS_INFO("(STATUS_SYSTEM) Set robot stage: %d",stage_);
        }
    }    

    //read volume
    n.getParam("volume_file", volumeFile);
    std::ifstream ifsVolume(volumeFile, std::ifstream::in);
    if(ifsVolume){
        ifsVolume >> volume_;
        ifsVolume.close();
        ROS_INFO("(STATUS_SYSTEM) Read robot volume: %d",volume_);
        std_msgs::Int8 data;
        data.data = volume_;
        volume_pub.publish(data);
        //set system sound volume
        SetRobot.changeVolume(volume_);
    } 

    //read loop
    n.getParam("path_loop_file", pathLoopFile);
    std::ifstream ifsLoop(pathLoopFile, std::ifstream::in);
    if(ifsLoop){
        ifsLoop >> loop_;
        ifsLoop.close();
        ROS_INFO("(STATUS_SYSTEM) Read robot path loop: %d",loop_);
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
        ROS_INFO("(STATUS_SYSTEM) Read robot path: %s",path_info.c_str());
    }

    //read name
    n.getParam("robot_name_file", nameFile);
    std::ifstream ifsName(nameFile, std::ifstream::in);
    if(ifsName){
        ifsName >> hostname_;
        ifsName.close();
        ROS_INFO("(STATUS_SYSTEM) Read robot name: %s",hostname_.c_str());
    }

    //read home
    n.getParam("home_file", homeFile);
    std::ifstream ifsHome(homeFile, std::ifstream::in);
    if(ifsHome){
        ifsHome >> home_.at(0) >> home_.at(1) >> home_.at(2) >> home_.at(3) >> home_.at(4) >> home_.at(5);
        ifsHome.close();
        ROS_INFO("(STATUS_SYSTEM) Read robot home: [%.2f, %.2f] [%.2f, %.2f, %.2f, %.2f]", std::stod(home_.at(0)),std::stod(home_.at(1)),std::stod(home_.at(2)),std::stod(home_.at(3)),std::stod(home_.at(4)),std::stod(home_.at(5)));
    }

    //read low battery
    n.getParam("low_battery_file", lowBatteryFile);
    std::ifstream ifsBattery(lowBatteryFile, std::ifstream::in);
    if(ifsBattery){
        ifsBattery >> low_battery_;
        ifsBattery.close();
        ROS_INFO("(STATUS_SYSTEM) Read robot battery: %d", std::stoi(low_battery_));
    }

    //read speed
    n.getParam("speed_file", speedFile);
    std::ifstream ifsSpeed(speedFile, std::ifstream::in);
    if(ifsSpeed){
        ifsSpeed >> linear_spd_ >> angular_spd_;
        ifsSpeed.close();
        ROS_INFO("(STATUS_SYSTEM) Read robot speed: %.2f, %.2f", std::stod(linear_spd_),std::stod(angular_spd_));
    }

    //read wifi
    n.getParam("wifi_file", wifiFile);
    n.getParam("deleteWIFI", deleteWifi);
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
    ROS_INFO("(STATUS_SYSTEM) Read robot wifi: name:%s, password:%s",wifi_.at(0).c_str(),wifi_.at(1).c_str());
    
    //record disconnect times
    n.getParam("disconnected_file", disconnectedFile);
    std::ofstream ofsDisconnected(disconnectedFile, std::ofstream::out | std::ofstream::trunc);
    if(ofsDisconnected){
        ofsDisconnected << disconnected;
        ofsDisconnected.close();
    }

    n.getParam("battery_log", recordBatteryFile);

    //set sound output device - if speaker is changed, need to change the audio_device parameters accordingly
    std::string audio_device;
    n.getParam("audio_device", audio_device);
    std::string cmd = "pacmd set-default-sink " + audio_device;
    system(cmd.c_str());
    ROS_INFO("(STATUS_SYSTEM) Set system sound output: %s",audio_device.c_str());
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "gobot_status");
    ros::NodeHandle n;

    SetRobot.initialize();
    volume_pub = n.advertise<std_msgs::Int8>("/gobot_status/volume", 1, true);
    update_info_pub = n.advertise<std_msgs::String>("/gobot_status/update_information", 1, true);
    status_pub = n.advertise<std_msgs::Int8>("/gobot_status/gobot_status", 1, true);

    initialData();

    ros::ServiceServer initializeHome = n.advertiseService("/gobot_recovery/initialize_home",initializeHomeSrcCallback);

    ros::ServiceServer setDockStatusSrv = n.advertiseService("/gobot_status/set_dock_status", setDockStatusSrvCallback);
    ros::ServiceServer getDockStatusSrv = n.advertiseService("/gobot_status/get_dock_status", getDockStatusSrvCallback);
    ros::ServiceServer isChargingSrv = n.advertiseService("/gobot_status/charging_status", isChargingSrvCallback);

    ros::ServiceServer setStageSrv = n.advertiseService("/gobot_status/set_stage", setStageSrvCallback);
    ros::ServiceServer getStageSrv = n.advertiseService("/gobot_status/get_stage", getStageSrvCallback);

    ros::ServiceServer setModeSrv = n.advertiseService("/gobot_status/set_mode", setModeSrvCallback);

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

    ros::ServiceServer setVolumeSrv = n.advertiseService("/gobot_status/set_volume", setVolumeSrvCallback);
    ros::ServiceServer getVolumeSrv = n.advertiseService("/gobot_status/get_volume", getVolumeSrvCallback);

    ros::ServiceServer setWifiSrv = n.advertiseService("/gobot_status/set_wifi", setWifiSrvCallback);
    ros::ServiceServer getWifiSrv = n.advertiseService("/gobot_status/get_wifi", getWifiSrvCallback);

    ros::ServiceServer PercentSrv = n.advertiseService("/gobot_status/battery_percent", PercentService);

    ros::ServiceServer getUpdateStatusSrv = n.advertiseService("/gobot_status/get_update_status", getUpdateStatusSrvCallback);

    ros::ServiceServer updateStatusSrv = n.advertiseService("/gobot_status/update_status", updateStatusSrvCb);

    ros::ServiceServer setGobotStatusSrv = n.advertiseService("/gobot_status/get_gobot_status", getGobotStatusSrvCallback);
    ros::ServiceServer getGobotStatusSrv = n.advertiseService("/gobot_status/set_gobot_status", setGobotStatusSrvCallback);
    
    ros::Subscriber battery_sub = n.subscribe("/gobot_base/battery_topic",1, batteryCallback);
    ros::Subscriber bumpers_sub = n.subscribe("/gobot_base/bumpers_topic",1, bumpersCallback);
    ros::Subscriber magnet_sub = n.subscribe("/gobot_base/magnet_topic", 1, magnetCb);

    ros::spin();

    return 0;
}
