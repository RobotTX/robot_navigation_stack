#include "gobot_software/tcp_command_system.hpp"

#define CMD_PORT 5600

const int max_length = 1024;

bool scanning = false;
bool simulation = false;

int robot_pos_port = 4001, map_port = 4002, laser_port = 4003;
std::string self_ip = "0.0.0.0";

std::vector<char> slient_command({'g','t','u','s','v'}),
                  scanning_command({'d','i','j','k','l','m','o'}),
                  docking_command({'d','j','k','o'}),
                  pause_path_command({'i','m','o','v','t','g'}),
                  cancel_resume_command({'d','g','i','j','k','l','m','p','t',','});

bool resume_path = false, resume_point = false;
std::vector<std::string> resume_pose;

/// Separator which is just a char(31) => unit separator in ASCII
static const std::string sep = std::string(1, 31);
static const char sep_c = 31;

std::mutex socketsMutex, commandMutex;

std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

std::string tts_text_;

std_srvs::Empty empty_srv;

ros::Publisher teleop_pub;

robot_class::SetRobot SetRobot;
robot_class::GetRobot GetRobot;
int robot_status_;
std::string status_text_;

template<typename Out>
void split(const std::string &s, const char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim) && ros::ok()) {
        *(result++) = item;
    }
}

bool execCommand(const std::string ip, const std::vector<std::string> command){
    std::string commandStr = command.at(0);
    bool status=false;
    
    GetRobot.getStatus(robot_status_,status_text_);
    //scanning
    if(robot_status_<=25 && robot_status_>=20){
        //pausePath, newPath, 
        //playPath, playPoint, 
        //stopPath, stop&deletePath,
        //goCharging
        if(std::find(scanning_command.begin(),scanning_command.end(),commandStr.at(0)) != scanning_command.end()){
            ROS_WARN("(COMMAND_SYSTEM) Gobot is scanning.");
            return false;
        }
    }
    //docking
    else if(robot_status_==15){
        //pausePath, playPath, 
        //playPoint, goCharging
        if(std::find(docking_command.begin(),docking_command.end(),commandStr.at(0)) != docking_command.end()){
            ROS_WARN("(COMMAND_SYSTEM) Gobot is docking.");
            return false;
        }
        //newCS, startScan
        else if(commandStr.at(0)=='n' || commandStr.at(0)=='t' || commandStr.at(0)=='g'){
            ros::service::call("/gobot_function/stopDocking", empty_srv);
        }
        //stopPath
        else if(commandStr.at(0)=='l'){
            ros::service::call("/gobot_function/stopDocking", empty_srv);
            return true;
        }
    }
    //moving
    else if(robot_status_==5){
        //newPath, stop&deletePath
        //goCharging,shutdownRobot
        //startScan
        if(std::find(pause_path_command.begin(),pause_path_command.end(),commandStr.at(0)) != pause_path_command.end()){
            ROS_WARN("(COMMAND_SYSTEM) Gobot is playing path,pause it.");
            ros::service::call("/gobot_function/pause_path", empty_srv);
        }
        //playPoint && stopPath, then pause path
        if(status_text_=="PLAY_POINT" && commandStr.at(0)=='l'){
            ROS_WARN("(COMMAND_SYSTEM) Gobot is playing point,pause it.");
            ros::service::call("/gobot_function/pause_path", empty_srv);
            return true;
        }
    }

    switch (commandStr.at(0)) {
        /// Interrupt delay/human action
        case '4':
            status = textToSpeech(command);
        break;
        /// Interrupt delay/human action
        case '3':
            status = interruptDelay(command);
        break;
        /// Adjust linear&angular speed
        case '1':
            status = adjustSpeed(command);
        break;
        /// Adjust auto-charging battery level
        case '2':
            status = adjustBatteryLvl(command);
        break;
        /// Command to change the name of the robot
        case 'a':
            status = renameRobot(command);
        break;

        /// Command for the robot to move to the previous point
        case 'b':
            status = previousPath(command);
        break;
        
        /// not in use
        /// Command for the robot to move to a point
        case 'c':
            status = newGoal(command);
        break;

        /// Command for the robot to pause the path
        case 'd':
            status = pausePath(command);
        break;

        /// Command for the robot to move to the next point
        case 'e':
            status = nextPath(command);
        break;

        /// Command for the robot to move to the previous point
        case 'f':
            status = pauseScan(ip, command);
        break;

        case 'g':
            status = startScanAndAutoExplore(ip, command);
        break;

        /// Command for the robot to receive the ports needed for the map and robot pos services
        case 'h':
            status = robotStartup(command);
        break;

        /// Command for the robot to save a new path
        case 'i':
            status = newPath(command);
        break;

        /// Command for the robot to play the saved path
        case 'j':
            status = playPath(command);
        break;

        /// Command for the robot to delete the saved path
        case 'k':
            status = playPoint(command);
        break;

        /// Command to stop the robot while following its path
        case 'l':
            status = stopPath(command);
        break;

        // command to stop the robot and then delete its path
        case 'm':
            status = stopAndDeletePath(command);
        break;

        // command to save the home of the robot
        case 'n':
            status = newChargingStation(command);
        break;

        // command to send the robot home
        case 'o':
            status = goToChargingStation(command);
        break;

        // command so that the robot stops on its way home
        case 'p':
            status = stopGoingToChargingStation(command);
        break;

        /// NOT USED ANYMORE
        /// Command to receive stored points
        case 'q':
            status = savePoints(command);
        break;

        /// NOT USED ANYMORE
        /// Command to stop sending the laser data to the Qt app
        case 'r':

        break;

        /// Command for the robot to send its map once
        case 's':
            status = sendMapOnce(ip, command);
        break;

        /// Command for the robot to start a scan from the beggining
        case 't':
            status = startNewScan(ip, command);
        break;

        /// Command for the robot to stop a scan
        case 'u':
            status = stopScanning(ip, command);
        break;

        /// command to shutdown robot
        case 'v':
            status = shutdownRobot(command);
        break;

        /// Not mute robot
        case 'w':
            status = muteOff(command);
        break;

        /// Mute robot
        case 'x':
            status = muteOn(command);
        break;

        /// command to set wifi
        case 'y':
            status = setWifi(command);
        break;

        case 'z':
            status = restartEverything(command);
        break;

        /// Command for the robot to start to explore the map automatically
        case ',':
            status = startAutoExplore(command);
        break;

        /// Command for the robot to stop to explore the map automatically
        case '.':
            status = stopAutoExplore(command);
        break;

        /// Command for the robot to loop the path
        case '/':
            status = loopPath(command);
        break;

        /// Default/Unknown command
        default:
            ROS_ERROR("(COMMAND_SYSTEM) Unknown command '%s' with %lu arguments : ", command.at(0).c_str(), command.size()-1);
            if(command.size() < 10){
                for(int i = 0; i < command.size(); i++)
                    ROS_ERROR("%s", command.at(i).c_str());
            } else 
                ROS_WARN("(COMMAND_SYSTEM) Too many arguments to display (%lu)", command.size());
        break;
    }
    return status;
}

/*********************************** COMMAND FUNCTIONS ***********************************/
// Command: 4, 2nd param=text to be converted to speech
bool textToSpeech(const std::vector<std::string> command){
    if(command.size() == 2){
        tts_text_ = command.at(1);
        SetRobot.speakChinese(tts_text_);
        return true;
    }
    return false;
}

// Command: 3, 2nd param=interrupt flag
bool interruptDelay(const std::vector<std::string> command){
    if(command.size() == 2){
        ros::service::call("/gobot_function/interrupt_delay",empty_srv);
        return true;
    }
    return false;
}

/// Command: 1, 2nd param=linear_vel, 3rd param=angular_vel
bool adjustSpeed(const std::vector<std::string> command){
    if(command.size() == 3){
        return SetRobot.setSpeedLimit(command.at(1),command.at(2));
    }
    return false;
}

/// Command: 2, 2nd param=battery level
bool adjustBatteryLvl(const std::vector<std::string> command){
    if(command.size() == 2){
        return SetRobot.setBatteryLvl(command.at(1));
    }
    return false;
}


/// Command : a, second param = new name 
bool renameRobot(const std::vector<std::string> command){
    if(command.size() == 2){
        //ROS_INFO("(COMMAND_SYSTEM) New name : %s", command.at(1).c_str());
        return SetRobot.setName(command.at(1));
    } 
    else 
        ROS_ERROR("(COMMAND_SYSTEM) Name missing");

    return false;
}

/// Command : b, still developing
bool previousPath(const std::vector<std::string> command){
    return false;
    if(command.size() == 1) {
        gobot_msg_srv::SetInt previous_path;
        previous_path.request.data = -1;
        return ros::service::call("/gobot_function/skip_path",previous_path);
    }
}

/// First param = c, second param = goal pos x coordinate, third param = goal pos y coordinate
bool newGoal(const std::vector<std::string> command){

        return true;
}

/// First param = d
bool pausePath(const std::vector<std::string> command){
    if(command.size() == 1) {
        if (ros::service::call("/gobot_function/pause_path", empty_srv)){
            //ROS_INFO("(COMMAND_SYSTEM) Pause the path");
            return true;
        }
    }

    return false;
}

/// First param = e
bool nextPath(const std::vector<std::string> command){
    if(command.size() == 1) {
        gobot_msg_srv::SetInt next_path;
        next_path.request.data = 1;
        return ros::service::call("/gobot_function/skip_path",next_path);
    }

    return false;
}
/// First param = f
bool pauseScan(const std::string ip, const std::vector<std::string> command){
    if(command.size() == 1){         
        //ROS_INFO("(COMMAND_SYSTEM) Gobot pause the ongoing scan");         
        std_srvs::Empty arg;         
        ros::service::call("/gobot_scan/stopExploration", arg);         
        return stopSendingMapAutomatically(ip);     
    }     
    return false;
}

/// First param = g
bool startScanAndAutoExplore(const std::string ip, const std::vector<std::string> command){
    if(command.size() == 1){     
        scanning = true;
        ROS_INFO("(COMMAND_SYSTEM::NewMap) Start Scan and Explore. We relaunched gobot_scan");
        /// Kill gobot move so that we'll restart it with the new map
        std::string cmd = SetRobot.killList();
        system(cmd.c_str());    
        /// Relaunch gobot_navigation
        SetRobot.runScan(simulation);

        /// 0 : the robot doesn't go back to its starting point at the end of the scan
        /// 1 : robot goes back to its starting point which is its charging station
        /// 2 : robot goes back to its starting point which is not a charging station
        gobot_msg_srv::SetInt exploration_srv;
        gobot_msg_srv::IsCharging isCharging;
        ros::service::call("/gobot_status/charging_status", isCharging);
        if(isCharging.response.isCharging){
            exploration_srv.request.data = 1;
        }
        else{
            exploration_srv.request.data = 0;
        }
        if(ros::service::call("/gobot_scan/startExploration", exploration_srv))
            return sendMapAutomatically(ip);
        else
            ROS_ERROR("(COMMAND_SYSTEM) Could not call the service /gobot_scan/startExploration");

    } else 
        ROS_ERROR("(COMMAND_SYSTEM) Parameter missing");

    return false;
}

/// First param = h, 2nd is port for robot position, 3rd for map, 4th for laser
bool robotStartup(const std::vector<std::string> command){
    /// TODO remove this
    if(command.size() == 4){
        robot_pos_port = std::stoi(command.at(1));
        map_port = std::stoi(command.at(2));
        laser_port = std::stoi(command.at(3));
        //ROS_INFO("(COMMAND_SYSTEM) Gobot here are the ports %d, %d, %d", robot_pos_port, map_port, laser_port);
        return true;
    } else
        ROS_ERROR("(COMMAND_SYSTEM) Parameter missing");

    return false;
}

/// First param = i, then the path name, then quadriplets of parameters to represent path points (path name, point name, posX, posY, waiting time,orientation) 
bool newPath(const std::vector<std::string> command){
    if(command.size() >= 6 && command.size()%7==2){  
        //ROS_INFO("(COMMAND_SYSTEM) New path received");
        gobot_msg_srv::SetStringArray set_path;
        for(int i = 1; i < command.size(); i++)
            set_path.request.data.push_back(command.at(i));

        if(ros::service::call("/gobot_status/set_path", set_path)){
            return true;
        } 
    } 
    else 
        ROS_ERROR("(COMMAND_SYSTEM) Parameter missing %lu %lu", command.size(), command.size()%5);

    return false;
}

/// First param = j
bool playPath(const std::vector<std::string> command){
    if(command.size() == 1) {
        if(ros::service::call("/gobot_function/play_path", empty_srv)){
            //ROS_INFO("(COMMAND_SYSTEM) Play the path");
            return true;
        }
    } 

    return false;
}

/// First param = k, 2nd is point name, 3rd is x coordinate, 4th is y coordinate, 5th is orientation, 6th is home bool
bool playPoint(const std::vector<std::string> command){
    if(command.size() == 6) {
        gobot_msg_srv::SetStringArray set_point;    
        set_point.request.data.push_back(command.at(1));
        set_point.request.data.push_back(command.at(2));
        set_point.request.data.push_back(command.at(3));
        set_point.request.data.push_back(command.at(4));
        set_point.request.data.push_back(command.at(5));
        if(ros::service::call("/gobot_function/play_point", set_point)){
            //ROS_INFO("(COMMAND_SYSTEM) Play the point");
            return true;
        }
    } 

    return false;
}

/// First param = l
bool stopPath(const std::vector<std::string> command){
    if(command.size() == 1) {
        //not stop docking
        if(ros::service::call("/gobot_function/stop_path", empty_srv)){
            ros::service::call("/move_base/clear_costmaps",empty_srv);
            //ROS_INFO("(COMMAND_SYSTEM) Stop the current motion");
            return true;
        }
    }

    return false;
}

/// First param = m
bool stopAndDeletePath(const std::vector<std::string> command){
    if(command.size() == 1) {
        //ROS_INFO("(COMMAND_SYSTEM) Stop the robot and delete its path");
        if(SetRobot.clearPath()){
            return true;
        }
    }

    return false;
}

/// First param = n, 2nd is the home x coordinate, 3rd is the home y coordinate, 4th is the orientation of the home
bool newChargingStation(const std::vector<std::string> command){
    if(command.size() == 4){
        // TODO send an angle from the application and convert it to store it in home.txt
        //ROS_INFO("(COMMAND_SYSTEM) Home received %s %s %s", command.at(1).c_str(), command.at(2).c_str(), command.at(3).c_str());
        
        //Check home from scanned map or new map
        std::string homeX = command.at(1), homeY = command.at(2), homeOri = command.at(3);
        int orientation = std::stoi(homeOri);
        tf::Quaternion quaternion;
        quaternion.setRPY(0, 0, -SetRobot.degreeToRad(orientation+90));

        std::string mapType = homeX.substr(0,1);
        if(mapType == "S"){
            homeX = homeX.substr(1);
            //ROS_INFO("(Command System::New CS) Map Type: %s", mapType.c_str());
            //We change the last known pose for localize Gobot in scanned map
        }
        SetRobot.setHome(homeX,homeY,std::to_string(quaternion.x()),std::to_string(quaternion.y()),std::to_string(quaternion.z()),std::to_string(quaternion.w()));
        ros::service::call("/gobot_recovery/initialize_home",empty_srv);
        return true;
    } 
    else
        ROS_ERROR("(COMMAND_SYSTEM) Not enough arguments, received %lu arguments, 4 arguments expected", command.size());

    return false;
}

/// First param = o
bool goToChargingStation(const std::vector<std::string> command){
    if(command.size() == 1){
        //if go to charging or charging, ignore
        if(GetRobot.getDock()==1){
            ros::service::call("/gobot_recovery/initialize_home",empty_srv);
            return false;
        }
        else if(GetRobot.getDock()==-3){
            ROS_INFO("(COMMAND_SYSTEM) Unable to go dock now because battery not stabilize yet.");
            return true;
        }
        else{
            //ROS_INFO("(COMMAND_SYSTEM) Sending the robot home");
            ros::service::call("/gobot_function/startDocking", empty_srv);
        }
    }
    return true;
}

/// First param = p
bool stopGoingToChargingStation(const std::vector<std::string> command){
    if(command.size() == 1) {
        //docking
        if(GetRobot.getDock()==3){
            //ROS_INFO("(COMMAND_SYSTEM) Stop sending the robot home");
            return ros::service::call("/gobot_function/stopDocking", empty_srv);
        }
    }

    return false;
}

/// First param = q
bool savePoints(const std::vector<std::string> command){

    return true;
}

/// First param = r

/// First param = s, second is who -> which widget requires it
bool sendMapOnce(const std::string ip, const std::vector<std::string> command){
    //ROS_INFO("(COMMAND_SYSTEM) Gobot send the map once");
    if(command.size() == 2){
        // std::stoi(command.at(1)):
        // 0 : scan 
        // 1 : application requesting at connection time
        // 2 : to merge

        gobot_msg_srv::SendMap srv;
        srv.request.who = std::stoi(command.at(1));
        srv.request.ip = ip;

        return ros::service::call("/gobot_function/send_once_map_sender", srv);
    } 
    else 
        ROS_ERROR("(COMMAND_SYSTEM) Not enough arguments, received %lu arguments, 2 arguments expected", command.size()); 

    return false;
}

/// First param = t
bool startNewScan(const std::string ip, const std::vector<std::string> command){
    if(command.size() == 1) {
        scanning = true;
        ROS_INFO("(COMMAND_SYSTEM::NewScan) Start New Scan. We relaunched gobot_scan");
        /// Kill gobot move so that we'll restart it with the new map
        std::string cmd = SetRobot.killList();
        system(cmd.c_str());
        /// Relaunch gobot_navigation
        SetRobot.runScan(simulation);

        return sendMapAutomatically(ip);
    }

    return false;
}

/// First param = u, 2nd is whether or not we want to kill gobot_move
bool stopScanning(const std::string ip, const std::vector<std::string> command){
    if(command.size() == 2) {
        ros::service::call("/gobot_scan/stopExploration", empty_srv);
        scanning = false;
        //if scan is cancelled, relaunch navigation here. if scan map is saved, relaunch navigation in tcp_read_map
        if(std::stoi(command.at(1)) == 1){
            ROS_INFO("(COMMAND_SYSTEM::StopScan) Stop New Scan. We relaunched gobot_navigation");
            /// Kill gobot move so that we'll restart it with the new map
            std::string cmd = SetRobot.killList();
            system(cmd.c_str());
 
            /// Relaunch gobot_navigation
            SetRobot.runNavi(simulation);
        }

        return stopSendingMapAutomatically(ip);
    }

    return false;
}

/// First param = v
bool shutdownRobot(const std::vector<std::string> command){
    ros::NodeHandle n;
    std::string shutdown_sh;
    if(n.hasParam("shutdown_file") && command.size() == 1){
        n.getParam("shutdown_file", shutdown_sh);
        //ROS_INFO("(COMMAND_SYSTEM) Shutdown Robot");
        if (ros::service::call("/gobot_base/shutdown_robot",empty_srv)){
            std::string cmd;
            cmd = "sudo sh " + shutdown_sh;
            system(cmd.c_str());
            return true;
        }
    }
    return false;
}

/// First param = w
bool muteOff(const std::vector<std::string> command){
    if(command.size() == 1) {
        //ROS_INFO("(COMMAND_SYSTEM) Disable mute");
        return SetRobot.setMute(0);
    }

    return false;
}

/// First param = x
bool muteOn(const std::vector<std::string> command){
    if(command.size() == 1) {
        //ROS_INFO("(COMMAND_SYSTEM) Enable mute");
        return SetRobot.setMute(1);
    }

    return false;
}

/// First param = y
bool setWifi(const std::vector<std::string> command){;
    //1=y, 2=wifi name, 3=wifi password
    if(command.size() == 3){
        //ROS_INFO("(COMMAND_SYSTEM) Wifi received %s %s", command.at(1).c_str(), command.at(2).c_str());
        return SetRobot.setWifi(command.at(1),command.at(2));
    } 
    else
        ROS_ERROR("(COMMAND_SYSTEM) Not enough arguments, received %lu arguments, 4 arguments expected", command.size());

    return false;
}

/// First param = z
bool restartEverything(const std::vector<std::string> command){
    ros::NodeHandle n;
    std::string restart_sh;
    if(n.hasParam("restart_file") && command.size() == 1){
        n.getParam("restart_file", restart_sh);
        //ROS_INFO("(COMMAND_SYSTEM) Gobot restarts its packages");
        //ROS_INFO("(COMMAND_SYSTEM) Please wait for 15s");
        std::string cmd = "sh " + restart_sh + " &";
        system(cmd.c_str());
        return true;
    }

    return false;
}

/// First param = ,
bool startAutoExplore(const std::vector<std::string> command){
    if(command.size() == 1) {
        //ROS_INFO("(COMMAND_SYSTEM) Start to explore");
        /// 0 : the robot doesn't go back to its starting point at the end of the scan
        /// 1 : robot goes back to its starting point which is its charging station
        /// 2 : robot goes back to its starting point which is not a charging station
        gobot_msg_srv::SetInt exploration_srv;
        gobot_msg_srv::IsCharging isCharging;
        ros::service::call("/gobot_status/charging_status", isCharging);
        if(isCharging.response.isCharging){
            exploration_srv.request.data = 1;
        }
        else{
            exploration_srv.request.data = 0;
        }
        if(ros::service::call("/gobot_scan/startExploration", exploration_srv))
            return true;
        else
            ROS_ERROR("(COMMAND_SYSTEM) Could not call the service /gobot_scan/startExploration");
    }

    return false;
}

/// First param = .
bool stopAutoExplore(const std::vector<std::string> command){
    if(command.size() == 1) {
        //ROS_INFO("(COMMAND_SYSTEM) Stop to explore");
        return ros::service::call("/gobot_scan/stopExploration", empty_srv);
    }

    return false;
}

/// First param = /, 2nd param is a bollean to know if we start or stop looping
bool loopPath(const std::vector<std::string> command){
    if(command.size() == 2) {
        //ROS_INFO("(COMMAND_SYSTEM) Loop the path %s", command.at(1).c_str());
        if(std::stoi(command.at(1)) == 0){
            ros::service::call("/gobot_function/stopLoopPath", empty_srv);
        } 
        else {
            ros::service::call("/gobot_function/startLoopPath", empty_srv);
        }

        return true;
    }

    return false;
}


/*********************************** SOME FUNCTIONS USED MULTIPLE TIMES ***********************************/
bool sendMapAutomatically(const std::string ip){
    //ROS_INFO("(COMMAND_SYSTEM) Launching the service to get the map auto");

    gobot_msg_srv::SetStringArray srv;
    srv.request.data.push_back(ip);

    if (ros::service::call("/gobot_function/send_auto_map_sender", srv)) {
        //~ROS_INFO("(COMMAND_SYSTEM) /gobot_function/send_auto_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(COMMAND_SYSTEM) Failed to call service /gobot_function/send_auto_map_sender");
        return false;
    }
}

bool stopSendingMapAutomatically(const std::string ip){
    //ROS_INFO("(COMMAND_SYSTEM) Launching the service to stop the map auto");

    gobot_msg_srv::SetStringArray srv;
    srv.request.data.push_back(ip);

    if (ros::service::call("/gobot_function/stop_auto_map_sender", srv)) {
        //~ROS_INFO("(COMMAND_SYSTEM) /gobot_function/stop_auto_map_sender service started");
        return true;
    } 
    else {
        ROS_ERROR("(COMMAND_SYSTEM) Failed to call service /gobot_function/stop_auto_map_sender");
        return false;
    }
}


void sendCommand(const std::string ip, const std::vector<std::string> command, std::string commandStr){
    //ROS_INFO("Command char:'%s', Command from ip: %s", command.at(0).c_str(),ip.c_str());

    //if command is given by user and robot is going to resume previous work after charging
    if(ip!=self_ip && (resume_path || resume_point)){
        //if given command can cancel the robot resume
        if(std::find(cancel_resume_command.begin(),cancel_resume_command.end(),commandStr.at(0)) != cancel_resume_command.end()){
            resume_path = false;
            resume_point = false;
            ROS_WARN("(COMMAND_SYSTEM) Cancel robot resume work due to user interruption");
        }
    }

    commandMutex.lock();
    bool command_feedback = execCommand(ip, command);
    if (command_feedback){
        if(std::find(slient_command.begin(),slient_command.end(),commandStr.at(0)) == slient_command.end())
            SetRobot.setSound(1,1);
    }

    std::string feedback_msg = (command_feedback ? "done" : "failed") + sep + commandStr;
    sendMessageToAll(feedback_msg);
    commandMutex.unlock();
}


/*********************************** SERVICES ***********************************/
bool pausePathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"d"});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);

    return true;
}

bool playPathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"j"});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);
    
    return true;
}

bool stopPathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"l"});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);

    return true;
}

bool startExploreSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({","});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);

    return true;
}

bool stopExploreSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"."});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);

    return true;
}

bool goDockSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"o"});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);
    return true;
}

bool stopGoDockSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"p"});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);
    return true;
}

bool highBatterySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(resume_path){
        ROS_WARN("(COMMAND_SYSTEM::HighBattery) robot resumes to play previous path.");
        resume_path = false;
        std::vector<std::string> command({"j"});
        std::string commandStr = command.at(0) + sep;
        sendCommand(self_ip,command,commandStr);
    }
    else if(resume_point){
        ROS_WARN("(COMMAND_SYSTEM::HighBattery) robot resumes to play previous position.");
        resume_point = false;
        if(resume_pose.size()==3){
            //0-name,1-x,2-y,3-orientation, 4-home or not
            std::vector<std::string> command({"k"});
            std::string commandStr = command.at(0) + sep;
            commandStr += "resume_point" + sep;
            command.push_back("resume_point");
            for (int i=0;i<resume_pose.size();i++){
                commandStr += resume_pose[i] + sep;
                command.push_back(resume_pose[i]);
            }
            commandStr += "0" + sep;
            command.push_back("0");
            sendCommand(self_ip,command,commandStr);
        }
    }
    else{
        ROS_WARN("(COMMAND_SYSTEM::HighBattery) robot resume to work has been cancelled before.");
    }

    return true;
}

bool lowBatterySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    GetRobot.getStatus(robot_status_,status_text_);
    //scanning process
    if(robot_status_<=25 && robot_status_>=20){
        ROS_WARN("(COMMAND_SYSTEM::LowBattery) robot is scanning.");
    } 
    //robot is docking
    else if(robot_status_==15){
        ROS_WARN("(COMMAND_SYSTEM::LowBattery) robot is docking.");
    }
    //robot is doing the path
    else if(ros::service::call("/gobot_function/goDockAfterPath", empty_srv)){
        resume_path = true;
        ROS_WARN("(COMMAND_SYSTEM::LowBattery) robot is go to auto dock after completing current path.");
    }
    else{
        ROS_WARN("(COMMAND_SYSTEM::LowBattery) robot is going to auto dock now.");
        resume_point = true;
        resume_pose.clear();
        gobot_msg_srv::GetStringArray robot_pose;
        ros::service::call("/gobot_status/get_pose",robot_pose);
        for(int i=0; i<robot_pose.response.data.size();i++)
            resume_pose.push_back(robot_pose.response.data[i]);
        resume_pose.back() = std::to_string(SetRobot.robotToAppYaw(std::stod(resume_pose.back()),"rad"));
        //ROS_INFO("(COMMAND_SYSTEM) Sending the robot home");
        std::vector<std::string> command({"o"});
        std::string commandStr = command.at(0) + sep;
        sendCommand(self_ip,command,commandStr);
    }

    return true;
}

bool playPointSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    std::vector<std::string> command({"k"});
    std::string commandStr = command.at(0) + sep;
    for (int i=0;i<req.data.size();i++){
        commandStr = commandStr + req.data[i] + sep;
        command.push_back(req.data[i]);
    }
    sendCommand(self_ip,command,commandStr);
    return true;
}

bool setPathSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    std::vector<std::string> command({"i"});
    std::string commandStr = command.at(0) + sep;
    for (int i=0;i<req.data.size();i++){
        commandStr = commandStr + req.data[i] + sep;
        command.push_back(req.data[i]);
    }
    sendCommand(self_ip,command,commandStr);
    return true;
}

bool setSpeedSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    std::vector<std::string> command({"1"});
    std::string commandStr = command.at(0) + sep;
    for (int i=0;i<req.data.size();i++){
        commandStr = commandStr + req.data[i] + sep;
        command.push_back(req.data[i]);
    }
    sendCommand(self_ip,command,commandStr);
    return true;
}

bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"v"});
    std::string commandStr = command.at(0) + sep;
    sendCommand(self_ip,command,commandStr);
    return true;
}


/*********************************** COMMUNICATION FUNCTIONS ***********************************/

void sendConnectionData(boost::shared_ptr<tcp::socket> sock){
    ros::NodeHandle n;
    /// Send a message to the app to tell we are connected
    gobot_msg_srv::GetStringArray get_speed;

    /// Get the charging station position from the home file
    std::string homeX, homeY;  
    double x,y,oriX,oriY,oriZ,oriW;

    GetRobot.getHome(x,y,oriX,oriY,oriZ,oriW);
    //if no home, set it to be (-150,-150) (invisible in the UI side)
    homeX = x==0? "-150" : std::to_string(x);
    homeY = y==0? "-150" : std::to_string(y);

    double homeOri = -SetRobot.radToDegree(tf::getYaw(tf::Quaternion(oriX , oriY , oriZ, oriW))) - 90.0;

    /// we send the path along with the time of the last modification of its file
    std::string path("");
    gobot_msg_srv::GetStringArray get_path;
	ros::service::call("/gobot_status/get_path", get_path);
    for(int i=0;i<get_path.response.data.size();i++){
        path += get_path.response.data.at(i) + sep;
    }

    /// we send the map id along with the time of the last modification of the map
    std::string mapId("");
    std::string mapDate("");
    std::string mapIdFile;
    if(n.hasParam("map_id_file")){
        n.getParam("map_id_file", mapIdFile);
        std::ifstream ifMap(mapIdFile, std::ifstream::in);
        if(ifMap){
            getline(ifMap, mapId);
            getline(ifMap, mapDate);
            ifMap.close();
        }
    }

    std::string laserStr("0");

    if(mapId.empty())
        mapId = "{5428014b-c5ec-4bec-9259-6c5a20517e0e}";
    if(mapDate.empty())
        mapDate = "1970-05-21-00-00-00";
    if(homeX.empty())
        homeX = "-1";
    if(homeY.empty())
        homeY = "-1";

    GetRobot.getStatus(robot_status_);
    std::string scan = (scanning) ? "1" : "0";
    std::string looping_str = (GetRobot.getLoop()) ? "1" : "0";
    std::string following_path_str = (robot_status_==5) ? "1" : "0";

    gobot_msg_srv::GetString get_battery;
    ros::service::call("/gobot_status/get_speed",get_speed);
    ros::service::call("/gobot_status/get_battery",get_battery);
    std::string linear_spd = get_speed.response.data[0],
                angular_spd=get_speed.response.data[1], 
                battery_lvl = get_battery.response.data;

    //Send robot information to UI once connected
    sendMessageToSock(sock, std::string("Connected" + sep + mapId + sep + mapDate + sep + homeX + sep + homeY + sep + std::to_string(homeOri) + 
                                       sep + scan + sep + laserStr + sep + following_path_str + sep + looping_str + sep + linear_spd + sep + 
                                       angular_spd + sep+ battery_lvl + sep +path));

    //Send more robot information to UI once connected
    ros::service::call("/gobot_status/update_status",empty_srv);
}

bool sendMessageToSock(boost::shared_ptr<tcp::socket> sock, const std::string message){
    std::string ip = sock->remote_endpoint().address().to_string();
    
    socketsMutex.lock();
    try {
        /// We send the result of the command to the given socket
        boost::asio::write(*sock, boost::asio::buffer(message, message.length()));

        //ROS_INFO("(COMMAND_SYSTEM) Message sent to %s succesfully",ip.c_str());
    } 
    catch (std::exception& e) {
        ROS_ERROR("(COMMAND_SYSTEM) Message not sent : %s", e.what());

    }
    socketsMutex.unlock();
}

bool sendMessageToAll(const std::string message){
    if(sockets.size() > 0){
        //ROS_INFO("(COMMAND_SYSTEM) Sending message : %s", message.c_str());
        socketsMutex.lock();
        for(auto const &elem : sockets){
            try {
                // We send the result of the command to every Qt app
                boost::asio::write(*(elem.second), boost::asio::buffer(message, message.length()));
            } 
            catch (std::exception& e) {
                // can not send msg to the ip, disconnect it
                ROS_ERROR("(COMMAND_SYSTEM) Message not sent:%s: %s",elem.first.c_str(),e.what());
            }
        }
        socketsMutex.unlock();
        return true;
    }
}

void session(boost::shared_ptr<tcp::socket> sock){
    std::string ip = sock->remote_endpoint().address().to_string();
    try {
        std::vector<std::string> command;
        std::string commandStr = "";
        bool finishedCmd = 0;

        /// Send some connection information to the server
        sendConnectionData(sock);
        /// Finally process any incoming command
        while(sockets.count(ip) && ros::ok()) {
            char data[max_length];
            boost::system::error_code error;
            /// We wait to receive some data
            size_t length = sock->read_some(boost::asio::buffer(data), error);
            if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
                ROS_WARN("(COMMAND_SYSTEM) Connection error %s. Closed", ip.c_str());
                return;
            } 
            else if (error) {
                throw boost::system::system_error(error); // Some other error.
                return;
            }

            for(int i = 0; i < length; i++){
                //Null
                if(static_cast<int>(data[i]) != 0){
                    //ETB
                    if(static_cast<int>(data[i]) == 23){
                        ROS_INFO("(COMMAND_SYSTEM) Completed reading command '%c' from ip '%s'", commandStr[0], ip.c_str());
                        finishedCmd = 1;
                        i = length;
                    } 
                    else{
                        commandStr += data[i];
                    }
                }
            }

            if(commandStr.length() > 0){
                /// Split the command from a str to a vector of str
                ///cmd+sep+param1+sep+param2+sep+ETB+sep
                split(commandStr, sep_c, std::back_inserter(command));

                if(finishedCmd){
                    sendCommand(ip,command,commandStr);
                    //reset variables
                    command.clear();
                    finishedCmd = 0;
                    commandStr = "";
                }
            } 
            else{
                ROS_ERROR("\n******************\n(COMMAND_SYSTEM) Got a bad command to debug :");
                std::istringstream iss2(data);

                std::string sub;
                while (iss2 && ros::ok()){
                    iss2 >> sub;
                }

                ROS_ERROR("(COMMAND_SYSTEM) data received : %lu byte(s) in str : %s", sub.length(), sub.c_str());
                for(int i = 0; i < max_length; i++)
                    if(static_cast<int>(data[i]) != 0)
                        ROS_ERROR("%d : %d or %c", i, static_cast<int>(data[i]), data[i]);

                ROS_ERROR("(COMMAND_SYSTEM) Stopping the function\n******************\n");
            }
        }
    } 
    catch (std::exception& e) {
        ROS_ERROR("(COMMAND_SYSTEM) Exception in thread, ip : %s => %s ", ip.c_str(), e.what());
    }

    ROS_INFO("(COMMAND_SYSTEM) Done with this session %s", ip.c_str());
}

void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), CMD_PORT));
    while(ros::ok()) {

        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));

        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        ROS_INFO("(COMMAND_SYSTEM) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();
 
        if(sockets.find(ip)==sockets.end()){ //not find the ip
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
        }
        else{   //ip exists in the list
            sockets.find(ip)->second->close();
            sockets.find(ip)->second = sock;
            //ROS_ERROR("(COMMAND_SYSTEM) the ip %s is already connected, this should not happen", ip.c_str());
        }
        socketsMutex.unlock();

        /// Launch the session thread which will communicate with the server
        std::thread(session, sock).detach();
    }

}
/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    disconnect(msg->data);
}

void disconnect(const std::string ip){
    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(ip)){
        sockets.at(ip)->close();
        sockets.erase(ip);
        ROS_WARN("(COMMAND_SYSTEM) The ip %s just disconnected", ip.c_str());
    }
    socketsMutex.unlock();
}

/*********************************** SHUT DOWN ***********************************/
void mySigintHandler(int sig){ 
    
    ros::shutdown();
}
/*********************************** MAIN ***********************************/

int main(int argc, char* argv[]){
    ros::init(argc, argv, "tcp_command_system");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    
    SetRobot.initialize();
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    //Startup end

    n.getParam("simulation", simulation);
    ROS_INFO("(COMMAND_SYSTEM) simulation : %d", simulation);
    
    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1, serverDisconnected);
    teleop_pub = n.advertise<geometry_msgs::Twist>("/teleop_cmd_vel", 1);
    
    ros::ServiceServer lowBatterySrv    = n.advertiseService("/gobot_command/lowBattery", lowBatterySrvCallback);
    ros::ServiceServer highBatterySrv   = n.advertiseService("/gobot_command/highBattery", highBatterySrvCallback);
    ros::ServiceServer goDockSrv        = n.advertiseService("/gobot_command/goDock", goDockSrvCallback);
    ros::ServiceServer stopGoDockSrv    = n.advertiseService("/gobot_command/stopGoDock", stopGoDockSrvCallback);
    ros::ServiceServer playPathSrv      = n.advertiseService("/gobot_command/play_path", playPathSrvCallback);
    ros::ServiceServer pausePathSrv     = n.advertiseService("/gobot_command/pause_path", pausePathSrvCallback);
    ros::ServiceServer stopPathSrv      = n.advertiseService("/gobot_command/stop_path", stopPathSrvCallback);
    ros::ServiceServer startExploreSrv  = n.advertiseService("/gobot_command/start_explore", startExploreSrvCallback);
    ros::ServiceServer stopExploreSrv   = n.advertiseService("/gobot_command/stop_explore", stopExploreSrvCallback);
    ros::ServiceServer playPointSrv     = n.advertiseService("/gobot_command/play_point", playPointSrvCallback);
    ros::ServiceServer setPathSrv       = n.advertiseService("/gobot_command/set_path", setPathSrvCallback);
    ros::ServiceServer shutdownSrv      = n.advertiseService("/gobot_command/shutdown", shutdownSrvCallback);
    ros::ServiceServer setSpeedSrv      = n.advertiseService("/gobot_command/set_speed", setSpeedSrvCallback);

    ROS_INFO("(COMMAND_SYSTEM) Ready to be launched.");

    std::thread t(server);
    
    ros::spin();
    return 0;
}
