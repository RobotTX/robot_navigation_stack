#include "command_system.hpp"

const int max_length = 1024;

bool waiting = false;
bool connected = false;
bool scanning = false;
bool recovering = false;
bool laserActivated = false;
bool simulation = false;

ros::Publisher go_pub;
ros::Publisher teleop_pub;

int robot_pos_port = 4001;
int map_port = 4002;
int laser_port = 4003;
int recovered_position_port = 4004;
int dockStatus = -2;
bool looping = false;

/// Separator which is just a char(31) => unit separator in ASCII
static const std::string sep = std::string(1, 31);
static const char sep_c = 31;

template<typename Out>
void split(const std::string &s, const char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

bool execCommand(const std::vector<std::string> command){

    std::string commandStr = command.at(0);
    bool status(false);
    switch (commandStr.at(0)) {

        /// Command to change the name of the robot
        case 'a':
            status = renameRobot(command);
        break;

        /// NOT USED ANYMORE
        /// Command to change the wifi of the robot
        case 'b':
            status = changeWifi(command);
        break;
        
        /// Command for the robot to move to a point
        case 'c':
            status = newGoal(command);
        break;

        /// Command for the robot to pause the path
        case 'd':
            status = pausePath(command);
        break;

        /// Command for the robot to play the ongoing scan
        case 'e':
            status = playScan(command);
        break;

        /// Command for the robot to pause the ongoing scan
        case 'f':
            status = pauseScan(command);
        break;

        case 'g':
            status = startScanAndAutoExplore(command);
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

        /// NOT USED ANYMORE
        /// Command for the robot to delete the saved path
        case 'k':
            status = deletePath(command);
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

        /// Command to send the laser data to the Qt app
        case 'q':
            status = sendLaserData(command);
        break;

        /// Command to stop sending the laser data to the Qt app
        case 'r':
            status = stopSendingLaserData(command);
        break;

        /// Command for the robot to send its map once
        case 's':
            status = sendMapOnce(command);
        break;

        /// Command for the robot to start a scan from the beggining
        case 't':
            status = startNewScan(command);
        break;

        /// Command for the robot to stop a scan
        case 'u':
            status = stopScanning(command);
        break;

        /// command to recover the robot's position
        case 'v':
            status = recoverPosition(command);
        break;

        /// command to pause during the recovery of a robot's position
        case 'w':
            status = pauseRecoveringPosition(command);
        break;

        /// command to stop recovering the robot's position
        case 'x':
            status = stopRecoveringPosition(command);
        break;

        case 'y':
            status = resumeRecoveringPosition(command);
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
            ROS_ERROR("(Command system) Unknown command '%s' with %lu arguments : ", command.at(0).c_str(), command.size()-1);
            if(command.size() < 10){
                for(int i = 0; i < command.size(); i++)
                    ROS_ERROR("%s", command.at(i).c_str());
            } else 
                ROS_WARN("(Command system) Too many arguments to display (%lu)", command.size());
        break;
    }
    return status;
}

/*********************************** COMMAND FUNCTIONS ***********************************/

/// Command : a, second param = new name 
bool renameRobot(const std::vector<std::string> command){
    ros::NodeHandle n;
    if(command.size() == 2){
        ROS_INFO("(Command system) New name : %s", command.at(1).c_str());

        std::string nameFile;
        if(n.hasParam("robot_name_file")){
            n.getParam("robot_name_file", nameFile);
            ROS_INFO("commandSystem set name file to %s", nameFile.c_str());
            std::ofstream ofs;
            ofs.open(nameFile, std::ofstream::out | std::ofstream::trunc);
            ofs << command.at(1);
            ofs.close();
            return true;
        }
    } else 
        ROS_ERROR("(Command system) Name missing");

    return false;
}

/// NOT USED ANYMORE
/// Command : b, second param = new ssid, third param = password
bool changeWifi(const std::vector<std::string> command){
    if(command.size() == 3){
        ROS_INFO("(Command system) New wifi : %s", command.at(1).c_str());
        ROS_INFO("(Command system) New wifi password : %s", command.at(2).c_str());
        std::string cmd = "sudo bash ~/computer_software/change_wifi.sh \"" + command.at(1) + "\" \""+ command.at(2) + "\"";
        ROS_INFO("(Command system) Cmd : %s", cmd.c_str());

        system(cmd.c_str());
        return true;
    } else 
        ROS_ERROR("(Command system) Parameter missing");

    return false;
}

/// First param = c, second param = goal pos x coordinate, third param = goal pos y coordinate
bool newGoal(const std::vector<std::string> command){
    ROS_INFO("(Command system) Gobot go to point");
    if(command.size() == 3){
        double posX = std::stof(command.at(1));
        double posY = std::stof(command.at(2));

        /// Before setting a new goal, we stop any teleoperation command
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        teleop_pub.publish(twist);

        ros::spinOnce();

        /// Send the goal
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = posX;
        msg.pose.position.y = posY;
        msg.pose.position.z = 0;
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 1;
        
        go_pub.publish(msg);

        dockStatus = 0;
        return true;
    } else 
        ROS_ERROR("(Command system) Parameter missing");

    return false;
}

/// First param = d
bool pausePath(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Pausing the path");
        std_srvs::Empty arg;
        if(ros::service::call("pause_path", arg)){
            ROS_INFO("(Command system) Pause path service called with success");
            return true;
        } else
            ROS_ERROR("(Command system) Pause path service call failed");
    }

    return false;
}

/// First param = e
bool playScan(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Gobot play the ongoing scan");
        return sendMapAutomatically();
    }

    return false;
}

/// First param = f
bool pauseScan(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Gobot pause the ongoing scan");

        std_srvs::Empty arg;
        ros::service::call("/move_base_controller/stopExploration", arg);

        return stopSendingMapAutomatically();
    }

    return false;
}

/// First param = g
bool startScanAndAutoExplore(const std::vector<std::string> command){
    if(command.size() == 1){    
        
        ROS_INFO("(Command system) Going to scan automatically");
        ROS_INFO("(Command system) Gobot start to scan a new map");
        scanning = true;

        /// Kill gobot move so that we'll restart it with the new map
        std::string cmd = "rosnode kill /move_base";
        system(cmd.c_str());

        sleep(5);
        /// Relaunch gobot_navigation
        if(simulation)
            cmd = "roslaunch gobot_navigation gazebo_scan.launch &";
        else
            cmd = "roslaunch gobot_navigation scan.launch &";
        system(cmd.c_str());
        ROS_INFO("(New Map) We relaunched gobot_navigation");

        sleep(2);

        /// 0 : the robot doesn't go back to its starting point at the end of the scan
        /// 1 : robot goes back to its starting point which is its charging station
        /// 2 : robot goes back to its starting point which is not a charging station
        hector_exploration_node::Exploration arg;
        arg.request.backToStartWhenFinished = 2;
        if(ros::service::call("/move_base_controller/startExploration", arg))
            return sendMapAutomatically();
        else
            ROS_ERROR("(Command system) Could not call the service /move_base_controller/startExploration");

    } else 
        ROS_ERROR("(Command system) Parameter missing");

    return false;
}

/// First param = h, 2nd is port for robot position, 3rd for map, 4th for laser
bool robotStartup(const std::vector<std::string> command){
    if(command.size() == 4){
        robot_pos_port = std::stoi(command.at(1));
        map_port = std::stoi(command.at(2));
        laser_port = std::stoi(command.at(3));
        ROS_INFO("(Command system) Gobot here are the ports %d, %d, %d", robot_pos_port, map_port, laser_port);
        startRobotPosConnection();
        startMapConnection();
        startLaserDataConnection(laserActivated);
        //connectToParticleCloudNode();
        return true;
    } else
        ROS_ERROR("(Command system) Parameter missing");

    return false;
}

/// First param = i, then the path name, then quadriplets of parameters to represent path points (path point name, posX, posY, waiting time) 
bool newPath(const std::vector<std::string> command){
    ros::NodeHandle n;
    if(command.size() >= 5 && command.size()%4 == 2){

        ROS_INFO("(Command system) Path received :");

        std::string pathFile;
        if(n.hasParam("path_file")){
            n.getParam("path_file", pathFile);
            ROS_INFO("commandSystem set path file to %s", pathFile.c_str());
            std::ofstream ofs(pathFile, std::ofstream::out | std::ofstream::trunc);
        
            if(ofs){
                std::string strPath;
                for(int i = 1; i < command.size(); i++){
                    ofs << command.at(i) << "\n";
                    strPath += sep + command.at(i);
                }

                ofs.close();
                
                // reset the path stage in the file
                std::string pathStageFile;
                if(n.hasParam("path_stage_file")){
                    n.getParam("path_stage_file", pathStageFile);
                    ROS_INFO("commandSystem set pathStageFile to %s", pathStageFile.c_str());
                }
                std::ofstream path_stage_file(pathStageFile, std::ofstream::out | std::ofstream::trunc);

                if(path_stage_file){
                    path_stage_file << "0";
                    path_stage_file.close();

                    std_srvs::Empty arg;
                    if(ros::service::call("stop_path", arg))
                        ROS_INFO("Stop path service called with success");
                    else
                        ROS_ERROR("Stop path service call failed");

                    return true;
                } else
                    ROS_ERROR("Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played");
            } else 
                ROS_ERROR("sorry could not open the file %s", pathFile.c_str());
        }
    } else 
        ROS_ERROR("(Command system) Parameter missing %lu %lu", command.size(), command.size()%4);

    return false;
}

/// First param = j
bool playPath(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Playing the path");
        std_srvs::Empty arg;
        if(ros::service::call("play_path", arg)){
            ROS_INFO("(Command system) Play path service called with success");
            dockStatus = 0;
            return true;
        } else
            ROS_ERROR("(Command system) Play path service call failed");
    }
    return false;
}

/// NOT USED ANYMORE
/// First param = k
bool deletePath(const std::vector<std::string> command){
    return false;
}

/// First param = l
bool stopPath(const std::vector<std::string> command){
    if(command.size() == 1) {
        
        ROS_INFO("(Command system) Stopping the path");
        std_srvs::Empty arg;
        if(ros::service::call("stop_path", arg)){
            ROS_INFO("(Command system) Stop path service called with success");
            return true;
        } else
            ROS_ERROR("(Command system) Stop path service call failed");
    }

    return false;
}

/// First param = m
bool stopAndDeletePath(const std::vector<std::string> command){
    ros::NodeHandle n;
    if(command.size() == 1) {
        ROS_INFO("(Command system) Stopping the robot and deleting its path");
        std_srvs::Empty arg;
        if(ros::service::call("stop_path", arg)){
            ROS_INFO("(Command system) Stop path service called with success");
            std::string pathFile;
            if(n.hasParam("path_file")){
                n.getParam("path_file", pathFile);
                ROS_INFO("(Command system) set path file to %s", pathFile.c_str());
                // deleting the path
                std::ofstream ofs(pathFile, std::ofstream::out | std::ofstream::trunc);
                ofs.close();
                return true;
            }
        } else
            ROS_ERROR("(Command system) Stop path service call failed");
    }

    return false;
}

/// First param = n, 2nd is the home x coordinate, 3rd is the home y coordinate, 4th is the orientation of the home
bool newChargingStation(const std::vector<std::string> command){
    ros::NodeHandle n;
    if(command.size() == 4){
        // TODO send an angle from the application and convert it to store it in home.txt
        ROS_INFO("(Command system) Home received %s %s %s", command.at(1).c_str(), command.at(2).c_str(), command.at(3).c_str());

        std::string homeFile;
        if(n.hasParam("home_file")){
            n.getParam("home_file", homeFile);
            ROS_INFO("(Command system) set home file to %s", homeFile.c_str());
            std::ofstream ofs(homeFile, std::ofstream::out | std::ofstream::trunc);
        
            if(ofs){
                int orientation = std::stoi(command.at(3));
                tf::Quaternion quaternion;
                quaternion.setEuler(0, 0, -(orientation+90)*3.14159/180);
                ofs << command.at(1) << " " << command.at(2) << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w();
                ofs.close();
                dockStatus = 0;
                return true;
            } else
                ROS_ERROR("(Command system) sorry could not open the file %s", homeFile.c_str());
        } else
            ROS_ERROR("(Command system) Could not find the param home_file");
    } else
        ROS_ERROR("(Command system) Not enough arguments, received %lu arguments, 4 arguments expected", command.size());

    return false;
}

/// First param = o
bool goToChargingStation(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Sending the robot home");
        std_srvs::Empty arg;

        if(ros::service::call("startDocking", arg)){
            ROS_INFO("(Command system) Go home service called with success");
            dockStatus = 3;
            return true;
        } else
            ROS_ERROR("(Command system) Stop path service call failed");
    }

    return false;
}

/// First param = p
bool stopGoingToChargingStation(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Stopping the robot on its way home");
        std_srvs::Empty arg;

        if(ros::service::call("stopDocking", arg)){
            ROS_INFO("(Command system) Stop going home service called with success");
            dockStatus = 0;
            return true;
        } else
            ROS_ERROR("(Command system) Stop going home service call failed");
    }

    return false;
}

/// First param = q
bool sendLaserData(const std::vector<std::string> command){
    ros::NodeHandle n;
    if(command.size() == 1) {
        ROS_INFO("(Command system) Gobot sends laser data");

        std_srvs::Empty srv;
        if(ros::service::call("send_laser_data_sender", srv)) {
            ROS_INFO("(Command system) send_laser_data_sender service started");
            std::string laserFile;
            if(n.hasParam("laser_file")){
                n.getParam("laser_file", laserFile);
                ROS_INFO("(Command system) set laser file to %s", laserFile.c_str());
                std::ofstream ofs;
                ofs.open(laserFile, std::ofstream::out | std::ofstream::trunc);
                ofs << "1";
                ofs.close();
            }

            return true;
        } else
            ROS_ERROR("(Command system) Failed to call service send_laser_data_sender");
        
    }

    return false;
}

/// First param = r
bool stopSendingLaserData(const std::vector<std::string> command){
    ros::NodeHandle n;
    if(command.size() == 1) {
        ROS_INFO("(Command system) Gobot stops sending laser data");

        std_srvs::Empty srv;

        if(ros::service::call("stop_send_laser_data_sender", srv)) {
            ROS_INFO("(Command system) stop_send_laser_data_sender service started");
            std::string laserFile;
            if(n.hasParam("laser_file")){
                n.getParam("laser_file", laserFile);
                ROS_INFO("(Command system) set laser file to %s", laserFile.c_str());
                std::ofstream ofs;
                ofs.open(laserFile, std::ofstream::out | std::ofstream::trunc);
                ofs << "0";
                ofs.close();
            }
            return true;
        } else
            ROS_ERROR("(Command system) Failed to call service stop_send_laser_data_sender");
    }

    return false;
}

/// First param = s, second is who -> which widget requires it
bool sendMapOnce(const std::vector<std::string> command){
    ROS_INFO("(Command system) Gobot send the map once");
    if(command.size() == 2){

        // std::stoi(command.at(1)):
        // 0 : scan 
        // 1 : application requesting at connection time
        // 2 : to merge
        // 3 : recovering position
        ROS_INFO("(Command system) Launching the service to get the map once");

        gobot_software::Port srv;
        srv.request.port = std::stoi(command.at(1));

        if (ros::service::call("send_once_map_sender", srv)) {
            ROS_INFO("(Command system) send_once_map_sender service started");
            return true;
        } else
            ROS_ERROR("(Command system) Failed to call service send_once_map_sender");
    } else 
        ROS_ERROR("(Command system) Not enough arguments, received %lu arguments, 2 arguments expected", command.size()); 

    return false;
}

/// First param = t
bool startNewScan(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Gobot start to scan a new map");
        scanning = true;

        /// Kill gobot move so that we'll restart it with the new map
        std::string cmd = "rosnode kill /move_base";
        system(cmd.c_str());

        sleep(5);
        /// Relaunch gobot_navigation
        if(simulation)
            cmd = "roslaunch gobot_navigation gazebo_scan.launch &";
        else
            cmd = "roslaunch gobot_navigation scan.launch &";
        system(cmd.c_str());
        ROS_INFO("(Command system) We relaunched gobot_navigation");

        return sendMapAutomatically();
    }

    return false;
}

/// First param = u, 2nd is whether or not we want to kill gobot_move
bool stopScanning(const std::vector<std::string> command){
    if(command.size() == 2) {
        ROS_INFO("(Command system) Gobot stops the scan of the new map");
        scanning = false;

        std_srvs::Empty arg;
        ros::service::call("/move_base_controller/stopExploration", arg);

        if(std::stoi(command.at(1)) == 1){

            /// Kill gobot move so that we'll restart it with the new map
            std::string cmd = "rosnode kill /move_base";
            system(cmd.c_str());
            sleep(5);

            /// Relaunch gobot_navigation
            if(simulation)
                cmd = "roslaunch gobot_navigation gazebo_slam.launch &";
            else
                cmd = "roslaunch gobot_navigation slam.launch &";
            system(cmd.c_str());
            ROS_INFO("(Command system) We relaunched gobot_navigation");
        }

        return stopSendingMapAutomatically();
    }

    return false;
}

/// First param = v
bool recoverPosition(const std::vector<std::string> command){
    /// TODO
    /*    
    ROS_INFO("(Command system) Launching the service to resume the recovery of the robot's position");
    std_srvs::Empty srv;

    if (ros::service::call("recover_position", srv)) {
        ROS_INFO("(Command system) resume recover_position started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to resume recover_position");
        return false;
    }
    */
    return false;
}

/// First param = w
bool pauseRecoveringPosition(const std::vector<std::string> command){
    /// TODO
    return false;
}

/// First param = x
bool stopRecoveringPosition(const std::vector<std::string> command){
    /// TODO
    /*  
    ROS_INFO("Stop recovering position called");
    std_srvs::Empty srv;
    ros::service::call("stop_checking_localization", srv);
    if(ros::service::call("stop_recovering_position", srv)){
        if(ros::service::call("stop_sending_local_map", srv)){
            ROS_INFO("(Command system) Stopped sending the local map");
            return true;
        }
        else {
            ROS_ERROR("(Command system) Could not stop sending the local map");
            return false;
        }
    } else { 
        ROS_ERROR("(Command system) Could not stop the recover position service");
        return false;
    }
    */
    return false;
}

/// First param = y
bool resumeRecoveringPosition(const std::vector<std::string> command){
    /// TODO
    /*    
    ROS_INFO("(Command system) Launching the service to resume the recovery of the robot's position");
    std_srvs::Empty srv;

    if (ros::service::call("recover_position", srv)) {
        ROS_INFO("(Command system) resume recover_position started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to resume recover_position");
        return false;
    }
    */
    return false;
}

/// First param = z
bool restartEverything(const std::vector<std::string> command){
    /// TODO finish/check if working
    if(command.size() == 1){
        ROS_INFO("(Command system) Gobot restarts its packages");
        recovering = false;
        scanning = false;
        system(("sh ~/computer_software/restart_packages.sh"));
        sleep(10);
        system(("sh ~/computer_software/roslaunch.sh"));

        return true;
    }

    return false;
}

/// First param = ,
bool startAutoExplore(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Start to explore");
        /// 0 : the robot doesn't go back to its starting point at the end of the scan
        /// 1 : robot goes back to its starting point which is its charging station
        /// 2 : robot goes back to its starting point which is not a charging station
        hector_exploration_node::Exploration arg;
        arg.request.backToStartWhenFinished = 2;
        if(ros::service::call("/move_base_controller/startExploration", arg))
            return true;
        else
            ROS_ERROR("(Command system) Could not call the service /move_base_controller/startExploration");
    }

    return false;
}

/// First param = .
bool stopAutoExplore(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Stop to explore");
        std_srvs::Empty arg;
        if(!ros::service::call("/move_base_controller/stopExploration", arg)){
            ROS_INFO("(Command system) Could not call the service /move_base_controller/stopExploration");
            return false;
        }

        return true;
    }

    return false;
}

/// First param = /, 2nd param is a bollean to know if we start or stop looping
bool loopPath(const std::vector<std::string> command){
    if(command.size() == 2) {
        ROS_INFO("(Command system) Loop the path %s", command.at(1).c_str());
        std_srvs::Empty arg;
        if(std::stoi(command.at(1)) == 0){
            if(ros::service::call("/stopLoopPath", arg)){
                looping = false;
                return true;
            } else
                ROS_ERROR("(Command system) Could not call the service /stopLoopPath");
        } else {
            if(ros::service::call("/startLoopPath", arg)){
                looping = true;
                return true;
            } else
                ROS_ERROR("(Command system) Could not call the service /startLoopPath");
        }
    }

    return false;
}


/*********************************** SOME FUNCTIONS USED MULTIPLE TIMES ***********************************/

bool sendMapAutomatically(void){
    ROS_INFO("(Command system) Launching the service to get the map auto");

    std_srvs::Empty srv;

    if (ros::service::call("send_auto_map_sender", srv)) {
        ROS_INFO("(Command system) send_auto_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service send_auto_map_sender");
        return false;
    }
}

bool stopSendingMapAutomatically(void){
    ROS_INFO("(Command system) Launching the service to stop the map auto");

    std_srvs::Empty srv;

    if (ros::service::call("stop_auto_map_sender", srv)) {
        ROS_INFO("(Command system) stop_auto_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service stop_auto_map_sender");
        return false;
    }
}


/*********************************** SERVICES ***********************************/

bool setDockStatus(gobot_software::SetDockStatus::Request &req, gobot_software::SetDockStatus::Response &res){
    ROS_INFO("(Command system) setDockStatus service called %d", req.status);
    dockStatus = req.status;

    return true;
}

bool getDockStatus(gobot_software::GetDockStatus::Request &req, gobot_software::GetDockStatus::Response &res){
    //ROS_INFO("(Command system) getDockStatus service called");
    res.status = dockStatus;

    return true;
}


/*********************************** STARTUP CONNECTION FUNCTIONS ***********************************/

void startRobotPosConnection(void){
    ROS_INFO("(Command system) Launching the service to get the robot position");

    gobot_software::Port srv;
    srv.request.port = robot_pos_port;

    if (ros::service::call("start_robot_pos_sender", srv))
        ROS_INFO("(Command system) start_robot_pos_sender service started");
    else 
        ROS_ERROR("(Command system) Failed to call service start_robot_pos_sender");
}

void stopRobotPosConnection(void){
    std_srvs::Empty srv;

    if (ros::service::call("stop_robot_pos_sender", srv)) 
        ROS_INFO("(Command system) stop_robot_pos_sender service started");
    else 
        ROS_ERROR("(Command system) Failed to call service stop_robot_pos_sender");
}


bool startMapConnection(void){
    ROS_INFO("(Command system) Launching the service to open the map socket");

    gobot_software::Port srv;
    srv.request.port = map_port;

    if (ros::service::call("start_map_sender", srv)) {
        ROS_INFO("(Command system) start_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service start_map_sender");
        return false;
    }
}

bool stopMapConnection(void){
    ROS_INFO("(Command system) stopMapConnection called");

    std_srvs::Empty srv;
    if (ros::service::call("stop_map_sender", srv)) {
        ROS_INFO("(Command system) stop_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service stop_map_sender");
        return false;
    }
}

bool startLaserDataConnection(const bool startLaser){
    ROS_INFO("(Command system) Launching the service which will send the lasers's data using port %d", laser_port);
    gobot_software::PortLaser srv;
    srv.request.port = laser_port;
    srv.request.startLaser = startLaser;

    if(ros::service::call("start_laser_data_sender", srv)) {
        ROS_INFO("(Command system) start_laser_data_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service start_laser_data_sender");
        return false;
    }
}

bool stopLaserDataConnection(void){
    std_srvs::Empty srv;
    if(ros::service::call("stop_laser_data_sender", srv)){
        ROS_INFO("Command system stop_sending_laser_data started");
        return true;
    } else {
        ROS_ERROR("(Command system) failed to call service stop_sending_laser_data");
        return false;
    }
}

/*********************************** COMMUNICATION FUNCTIONS ***********************************/

void getPorts(boost::shared_ptr<tcp::socket> sock){

    ROS_INFO("(Command system) getPorts launched");
    std::vector<std::string> command;
    std::string commandStr = "";
    char data[max_length];
    bool finishedCmd = 0;

    boost::system::error_code error;
    size_t length = sock->read_some(boost::asio::buffer(data), error);
    ROS_INFO("%lu byte(s) received", length);

    if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
        ROS_INFO("(Command system) Connection closed");
        connected = false;
        return;
    } else if (error) 
        throw boost::system::system_error(error); // Some other error.

    for(int i = 0; i < length; i++){
        if(static_cast<int>(data[i]) != 0){
            if(static_cast<int>(data[i]) == 23){
                ROS_INFO("(Command system) Command complete");
                finishedCmd = 1;
                i = length;
            } else
                commandStr += data[i];
        }
    }
    ROS_INFO("(Command system) Received : %s", commandStr.c_str());

    /// Split the command from a str to a vector of str
    split(commandStr, sep_c, std::back_inserter(command));

    if(finishedCmd){
        ROS_INFO("(Command system) Executing command : ");
        if(command.size() < 10){
            for(int i = 0; i < command.size(); i++)
                ROS_INFO("'%s'", command.at(i).c_str());
        } else 
            ROS_WARN("(Command system) Too many arguments to display (%lu)", command.size());

        execCommand(command);
        ROS_INFO("(Command system) GetPorts done");
    }
}

void session(boost::shared_ptr<tcp::socket> sock){
    ROS_INFO("(Command system) Waiting for a command");
    try{
        std::vector<std::string> command;
        std::string commandStr = "";
        bool finishedCmd = 0;

        getPorts(sock);

        while(ros::ok() && connected){
            char data[max_length] = {0};

            boost::system::error_code error;
            size_t length = sock->read_some(boost::asio::buffer(data), error);
            ROS_INFO("(Command system) %lu byte(s) received", length);
            if (error == boost::asio::error::eof)
                ROS_ERROR("(Command system) Got error eof");
            
            if (error == boost::asio::error::connection_reset){
                ROS_ERROR("(Command system) Connection closed");
                disconnect();
            } else if (error) 
                throw boost::system::system_error(error); // Some other error.

            for(int i = 0; i < length; i++){
                if(static_cast<int>(data[i]) != 0){
                    if(static_cast<int>(data[i]) == 23){
                        ROS_INFO("(Command system) Command complete");
                        finishedCmd = 1;
                        i = length;
                    } else
                        commandStr += data[i];
                }
            }

            if(commandStr.length() > 0){

                /// Split the command from a str to a vector of str
                split(commandStr, sep_c, std::back_inserter(command));

                if(finishedCmd){
                    ROS_INFO("(Command system) Executing command : ");
                    if(command.size() < 10){
                        for(int i = 0; i < command.size(); i++)
                            ROS_INFO("'%s'", command.at(i).c_str());
                    } else 
                        ROS_WARN("(Command system) Too many arguments to display (%lu)", command.size());

                    std::string msg = (execCommand(command) ? "done" : "failed") + sep + commandStr;
                    sendMessageToPc(sock, msg);
                    command.clear();
                    finishedCmd = 0;
                    commandStr = "";
                }
            } else {
                ROS_ERROR("\n******************\n(Command system) Got a bad command to debug :");
                std::istringstream iss2(data);

                std::string sub;
                while (iss2){
                    iss2 >> sub;
                }

                ROS_ERROR("(Command system) data received : %lu byte(s) in str : %s", sub.length(), sub.c_str());
                for(int i = 0; i < max_length; i++)
                    if(static_cast<int>(data[i]) != 0)
                        ROS_ERROR("%d : %d or %c", i, static_cast<int>(data[i]), data[i]);


                ROS_ERROR("(Command system) Stopping the function\n******************\n");
            }
        }
    } catch (std::exception& e) {
        ROS_ERROR("(Command system) Exception in thread: %s", e.what());
    }
}

bool sendMessageToPc(boost::shared_ptr<tcp::socket> sock, const std::string message){
    ROS_INFO("(Command system) Sending message : %s", message.c_str());

    try {
        boost::system::error_code ignored_error;
        boost::asio::write(*sock, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), ignored_error);
        
        ROS_INFO("(Command system) Message sent succesfully");
        return true;
    } catch (std::exception& e) {
        ROS_ERROR("(Command system) Message not sent : %s", e.what());
        return false;
    }
}

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor){
    ROS_INFO("(Command system) Waiting for connection");
    ros::NodeHandle n;

    boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(*io_service));

    m_acceptor->accept(*sock);
    ROS_INFO("(Command system) Command socket connected to %s", sock->remote_endpoint().address().to_string().c_str());
    connected = true;
    waiting = false;

    /// Send a message to the PC to tell we are connected
    /// send home position and timestamp
    std::string homeX("");
    std::string homeY("");
    double x_angle(0);
    double y_angle(0);
    double z_angle(0);
    double w_angle(1);
    tfScalar roll;
    tfScalar pitch;
    tfScalar yaw;
    std::string homeFile;
    double homeOri = 0;
    if(n.hasParam("home_file")){
        n.getParam("home_file", homeFile);
        ROS_INFO("(Command system) set homefile to %s", homeFile.c_str());
        std::ifstream ifs(homeFile, std::ifstream::in);
        
        if(ifs){
            ifs >> homeX >> homeY >> x_angle >> y_angle >> z_angle >> w_angle;

            tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(x_angle , y_angle , z_angle, w_angle));

            matrix.getRPY(roll, pitch, yaw);
            homeOri = -(yaw*180/3.14159) - 90;//-(orientation+90)*3.14159/180);
            ROS_INFO("(Command system) rotation %f", homeOri);

            ROS_INFO("(Command system) Home : [%s, %s, %f]", homeX.c_str(), homeY.c_str(), homeOri);
            dockStatus = 0;
            ifs.close();
        }
    }

    /// we also send the path along with the time of the last modification of its file
    std::string path("");
    std::string pathFile;
    if(n.hasParam("path_file")){
        n.getParam("path_file", pathFile);
        ROS_INFO("(Command system) set path file to %s", pathFile.c_str());
        std::ifstream ifPath(pathFile, std::ifstream::in);
        
        if(ifPath){
            std::string line("");
            ROS_INFO("(Command system) Line path");
            while(getline(ifPath, line))
                path += line + sep;
            ifPath.close();
        }
    }

    /// we also send the map id along with the time of the last modification of the map
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
    std::string laserFile;
    if(n.hasParam("laser_file")){
        n.getParam("laser_file", laserFile);
        std::ifstream ifLaser(laserFile, std::ifstream::in);
        
        if(ifLaser){
            getline(ifLaser, laserStr);
            ROS_INFO("(Command system) Laser activated : %s", laserStr.c_str());
            laserActivated = boost::lexical_cast<bool>(laserStr);
            ifLaser.close();
        }
    }

    if(mapId.empty())
        mapId = "{00000000-0000-0000-0000-000000000000}";
    if(mapDate.empty())
        mapDate = "1970-05-21-00-00-00";
    if(homeX.empty())
        homeX = "-150";
    if(homeY.empty())
        homeY = "-150";

    std::string scan = (scanning) ? "1" : "0";
    std::string recover = (recovering) ? "1" : "0";
    std::string looping_str = (looping) ? "1" : "0";

    sendMessageToPc(sock, "Connected" + sep + mapId + sep + mapDate + sep + homeX + sep + homeY + sep + std::to_string(homeOri) + sep
        + scan + sep + recover + sep + laserStr + sep + looping_str + sep + path);

    boost::thread t(boost::bind(session, sock));
}

void server(const unsigned short port){

    boost::shared_ptr<boost::asio::io_service> io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
    io_service->run();

    boost::shared_ptr<tcp::endpoint> m_endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), port));
    boost::shared_ptr<tcp::acceptor> m_acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*io_service, *m_endpoint));

    m_acceptor->set_option(tcp::acceptor::reuse_address(true));

    ros::Rate r(10);
    while(ros::ok()){
        if(!connected && !waiting){
            ROS_INFO("(Command system) Ready to connect");
            boost::thread t(boost::bind(asyncAccept, io_service, m_acceptor));

            waiting = true;
        }
        ros::spinOnce();
        r.sleep();
    }
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    disconnect();
}

void disconnect(void){
    if(connected){
        ROS_INFO("(Command system) Robot could not find the Qt application");
        stopRobotPosConnection();
        stopMapConnection();
        stopLaserDataConnection();
        connected = false;
    }
}

int main(int argc, char* argv[]){

    try{
        ros::init(argc, argv, "command_system");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);

        go_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        teleop_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        ros::ServiceServer setDockStatusSrv = n.advertiseService("setDockStatus", setDockStatus);
        ros::ServiceServer getDockStatusSrv = n.advertiseService("getDockStatus", getDockStatus);

        n.param<bool>("simulation", simulation, false);
        ROS_INFO("(New Map) simulation : %d", simulation);

        ros::spinOnce();

        server(CMD_PORT);
        
    } catch (std::exception& e) {
        ROS_ERROR("(Command system) Exception : %s", e.what());
    }

    return 0;
}
