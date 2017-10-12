#include "gobot_software/command_system_new.hpp"

const int max_length = 1024;

bool scanning = false;
std::string scanningIp;
bool laserActivated = false;
bool simulation = false;
bool looping = false;
bool low_battery = false;

ros::Publisher go_pub;
ros::Publisher teleop_pub;

int robot_pos_port = 4001;
int map_port = 4002;
int laser_port = 4003;
int recovered_position_port = 4004;
int dockStatus = -2;

std::mutex socketsMutex;
std::mutex commandMutex;

std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

/// Separator which is just a char(31) => unit separator in ASCII
static const std::string sep = std::string(1, 31);
static const char sep_c = 31;

gobot_msg_srv::SetGobotStatus set_gobot_status;
gobot_msg_srv::GetGobotStatus get_gobot_status;

ros::ServiceClient setGobotStatusSrv,getGobotStatusSrv;

template<typename Out>
void split(const std::string &s, const char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

bool execCommand(const std::string ip, const std::vector<std::string> command){

    std::string commandStr = command.at(0);
    bool status=false;
    switch (commandStr.at(0)) {

        /// Command to change the name of the robot
        case 'a':
            status = renameRobot(command);
        break;

        /// NOT USED ANYMORE
        /// Command to change the wifi of the robot
        case 'b':

        break;
        
        /// Command for the robot to move to a point
        case 'c':
            status = newGoal(command);
        break;

        /// Command for the robot to pause the path
        case 'd':
            status = pausePath(command);
        break;

        /// NOT USED ANYMORE
        /// Command for the robot to play the ongoing scan
        case 'e':

        break;

        /// NOT USED ANYMORE
        /// Command for the robot to pause the ongoing scan
        case 'f':

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

        /// NOT USED ANYMORE
        /// Command for the robot to delete the saved path
        case 'k':

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
        /// Command to send the laser data to the Qt app
        case 'q':

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

        /// NOT USED NOW
        /// command to recover the robot's position
        case 'v':

        break;

        /// NOT USED NOW
        /// command to pause during the recovery of a robot's position
        case 'w':

        break;

        /// NOT USED NOW
        /// command to stop recovering the robot's position
        case 'x':

        break;

        /// NOT USED NOW
        case 'y':

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
        if(ros::service::call("/gobot_function/pause_path", arg)){
            ROS_INFO("(Command system) Pause path service called with success");
            return true;
        } else
            ROS_ERROR("(Command system) Pause path service call failed");
    }

    return false;
}

/// First param = e

/// First param = f


/// First param = g
bool startScanAndAutoExplore(const std::string ip, const std::vector<std::string> command){
    if(command.size() == 1){    
        
        ROS_INFO("(Command system) Going to scan automatically");
        ROS_INFO("(Command system) Gobot start to scan a new map");
        scanning = true;
        scanningIp = ip;

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
        if(ros::service::call("/gobot_scan/startExploration", arg))
            return sendMapAutomatically(ip);
        else
            ROS_ERROR("(Command system) Could not call the service /gobot_scan/startExploration");

    } else 
        ROS_ERROR("(Command system) Parameter missing");

    return false;
}

/// First param = h, 2nd is port for robot position, 3rd for map, 4th for laser
bool robotStartup(const std::vector<std::string> command){
    /// TODO remove this
    if(command.size() == 4){
        robot_pos_port = std::stoi(command.at(1));
        map_port = std::stoi(command.at(2));
        laser_port = std::stoi(command.at(3));
        ROS_INFO("(Command system) Gobot here are the ports %d, %d, %d", robot_pos_port, map_port, laser_port);
        return true;
    } else
        ROS_ERROR("(Command system) Parameter missing");

    return false;
}

/// First param = i, then the path name, then quadriplets of parameters to represent path points (path point name, posX, posY, waiting time,yaw) 
bool newPath(const std::vector<std::string> command){
    ros::NodeHandle n;
    if(command.size() >= 6 && command.size()%5 == 2){

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
                    if(ros::service::call("/gobot_function/stop_path", arg)){
                        ROS_INFO("Stop path service called with success");
                    } else
                        ROS_ERROR("Stop path service call failed");

                    return true;
                } else
                    ROS_ERROR("Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played");
            } else 
                ROS_ERROR("sorry could not open the file %s", pathFile.c_str());
        }
    } else 
        ROS_ERROR("(Command system) Parameter missing %lu %lu", command.size(), command.size()%5);

    return false;
}

/// First param = j
bool playPath(const std::vector<std::string> command){
    if(command.size() == 1) {
        getGobotStatusSrv.call(get_gobot_status);
        if(get_gobot_status.response.status!=15){
            ROS_INFO("(Command system) Playing the path");
            std_srvs::Empty arg;
            if(ros::service::call("/gobot_function/play_path", arg)){
                ROS_INFO("(Command system) Play path service called with success");
                dockStatus = 0;
                return true;
            } else
                ROS_ERROR("(Command system) Play path service call failed");
        } else 
            ROS_ERROR("(Command system) Wait for the robot to dock before launching a new path");
    }
    return false;
}

/// NOT USED ANYMORE
/// First param = k


/// First param = l
bool stopPath(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Stopping the current motion");
        std_srvs::Empty arg;
        if(ros::service::call("/gobot_function/stop_path", arg)){
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

        getGobotStatusSrv.call(get_gobot_status);
        if(get_gobot_status.response.status!=15){

            ROS_INFO("(Command system) Stopping the robot and deleting its path");
            std_srvs::Empty arg;
            if(ros::service::call("/gobot_function/stop_path", arg)){
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
        } else 
            ROS_ERROR("(Command system) Wait for the robot to dock before launching a new path");
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
    if(command.size() == 1){

        ROS_INFO("(Command system) Sending the robot home");
        return goDock();
    }
    return false;
}

/// First param = p
bool stopGoingToChargingStation(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Stopping the robot on its way home");
        std_srvs::Empty arg;

        if(!low_battery && ros::service::call("/gobot_function/stopDocking", arg)){
            ROS_INFO("(Command system) Stop going home service called with success");
            dockStatus = 0;
            return true;
        } else
            ROS_ERROR("(Command system) Stop going home service call failed");
    }

    return false;
}

/// First param = q


/// First param = r

/// First param = s, second is who -> which widget requires it
bool sendMapOnce(const std::string ip, const std::vector<std::string> command){
    ROS_INFO("(Command system) Gobot send the map once");
    if(command.size() == 2){

        // std::stoi(command.at(1)):
        // 0 : scan 
        // 1 : application requesting at connection time
        // 2 : to merge
        ROS_INFO("(Command system) Launching the service to get the map once");

        gobot_msg_srv::SendMap srv;
        srv.request.who = std::stoi(command.at(1));
        srv.request.ip = ip;

        if (ros::service::call("/gobot_function/send_once_map_sender", srv)) {
            ROS_INFO("(Command system) /gobot_function/send_once_map_sender service started");
            return true;
        } else
            ROS_ERROR("(Command system) Failed to call service /gobot_function/send_once_map_sender");
    } else 
        ROS_ERROR("(Command system) Not enough arguments, received %lu arguments, 2 arguments expected", command.size()); 

    return false;
}

/// First param = t
bool startNewScan(const std::string ip, const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Gobot start to scan a new map");
        scanning = true;
        scanningIp = ip;

        /// Kill gobot move so that we'll restart it with the new map
        std::string cmd = "rosnode kill /move_base";
        system(cmd.c_str());

        sleep(5);
        /// Relaunch gobot_navigation
        if(simulation)
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gazebo_scan.launch\"";
        else
            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation scan.launch\"";
        system(cmd.c_str());
        ROS_INFO("(Command system) We relaunched gobot_navigation");

        return sendMapAutomatically(ip);
    }

    return false;
}

/// First param = u, 2nd is whether or not we want to kill gobot_move
bool stopScanning(const std::string ip, const std::vector<std::string> command){
    if(command.size() == 2) {
        ROS_INFO("(Command system) Gobot stops the scan of the new map");
        scanning = false;
        scanningIp = "";

        std_srvs::Empty arg;
        ros::service::call("/gobot_scan/stopExploration", arg);

        if(std::stoi(command.at(1)) == 1){

            /// Kill gobot move so that we'll restart it with the new map
            std::string cmd = "rosnode kill /move_base";
            system(cmd.c_str());
            sleep(5);

            /// Relaunch gobot_navigation
            if(simulation)
                cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gazebo_slam.launch\"";
            else
                cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation navigation.launch\"";
            system(cmd.c_str());
            ROS_INFO("(Command system) We relaunched gobot_navigation");
        }

        return stopSendingMapAutomatically(ip);
    }

    return false;
}

/// First param = v


/// First param = w


/// First param = x


/// First param = y


/// First param = z
bool restartEverything(const std::vector<std::string> command){
    ros::NodeHandle n;
    std::string restart_sh;
    if(n.hasParam("restart_file") && command.size() == 1){
        n.getParam("restart_file", restart_sh);
        ROS_INFO("(Command system) Gobot restarts its packages");
        ROS_INFO("(Command system) Please wait for 15s");
        scanning = false;
        scanningIp = "";
        std::string cmd = "sh"+restart_sh;
        system(cmd.c_str());
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
        if(ros::service::call("/gobot_scan/startExploration", arg))
            return true;
        else
            ROS_ERROR("(Command system) Could not call the service /gobot_scan/startExploration");
    }

    return false;
}

/// First param = .
bool stopAutoExplore(const std::vector<std::string> command){
    if(command.size() == 1) {
        ROS_INFO("(Command system) Stop to explore");
        std_srvs::Empty arg;
        if(!ros::service::call("/gobot_scan/stopExploration", arg)){
            ROS_INFO("(Command system) Could not call the service /gobot_scan/stopExploration");
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
            if(ros::service::call("/gobot_function/stopLoopPath", arg)){
                looping = false;
                return true;
            } else
                ROS_ERROR("(Command system) Could not call the service /gobot_function/stopLoopPath");
        } else {
            if(ros::service::call("/gobot_function/startLoopPath", arg)){
                looping = true;
                return true;
            } else
                ROS_ERROR("(Command system) Could not call the service /gobot_function/startLoopPath");
        }
    }

    return false;
}


/*********************************** SOME FUNCTIONS USED MULTIPLE TIMES ***********************************/

bool sendMapAutomatically(const std::string ip){
    ROS_INFO("(Command system) Launching the service to get the map auto");

    gobot_msg_srv::String srv;
    srv.request.data = ip;

    if (ros::service::call("/gobot_function/send_auto_map_sender", srv)) {
        ROS_INFO("(Command system) /gobot_function/send_auto_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service /gobot_function/send_auto_map_sender");
        return false;
    }
}

bool stopSendingMapAutomatically(const std::string ip){
    ROS_INFO("(Command system) Launching the service to stop the map auto");

    gobot_msg_srv::String srv;
    srv.request.data = ip;

    if (ros::service::call("/gobot_function/stop_auto_map_sender", srv)) {
        ROS_INFO("(Command system) /gobot_function/stop_auto_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service /gobot_function/stop_auto_map_sender");
        return false;
    }
}

bool goDock(void){
    std_srvs::Empty arg;

    if(ros::service::call("/gobot_function/startDocking", arg)){
        ROS_INFO("(Command system) /gobot_function/startDocking service called with success");
        dockStatus = 3;
        return true;
    } else {
        ROS_ERROR("(Command system) /gobot_function/startDocking service call failed");
        return false;
    }
}


void sendCommand(const std::string ip, const std::vector<std::string> command, std::string commandStr){
    //ROS_INFO("(Command system) Executing command for ip: %s", ip.c_str());
    ROS_INFO("Command char:'%s'", command.at(0).c_str());

    std::string msg;
    commandMutex.lock();
    msg = (execCommand(ip, command) ? "done" : "failed") + sep + commandStr;
    sendMessageToAll(msg);
    commandMutex.unlock();
}


void checkCommand(char c){
    //Before sending, double check command
    switch (c) {
        /// new path
        case 'i':
            getGobotStatusSrv.call(get_gobot_status);
            if(get_gobot_status.response.status==10 || get_gobot_status.response.status==5){
                std::vector<std::string> command({"d"});
                std::string commandStr = command.at(0) + sep;
                sendCommand("",command,commandStr);
            }
            /*
            if(looping){
                std::vector<std::string> command({"/","0"});
                std::string commandStr = command.at(0) + sep + command.at(1) + sep;
                sendCommand("",command,commandStr);
            }   
            */
        break;

        /// stop path
        case 'l':
            getGobotStatusSrv.call(get_gobot_status);
            if(get_gobot_status.response.status==15){
                std::vector<std::string> command({"p"});
                std::string commandStr = command.at(0) + sep;
                sendCommand("",command,commandStr);
            }
            else if(get_gobot_status.response.status==10 || get_gobot_status.response.status==5){
                std::vector<std::string> command({"d"});
                std::string commandStr = command.at(0) + sep;
                sendCommand("",command,commandStr);
            }
        break;

        /// delete and stop path
        case 'm':
            getGobotStatusSrv.call(get_gobot_status);
            if(get_gobot_status.response.status==10 || get_gobot_status.response.status==5){
                std::vector<std::string> command({"d"});
                std::string commandStr = command.at(0) + sep;
                sendCommand("",command,commandStr);
            }
        break;

        /// new charging station
        case 'n':
            getGobotStatusSrv.call(get_gobot_status);
            if(get_gobot_status.response.status==15){
                std::vector<std::string> command({"p"});
                std::string commandStr = command.at(0) + sep;
                sendCommand("",command,commandStr);
            }
        break;

        /// go to charging station
        case 'o':
            getGobotStatusSrv.call(get_gobot_status);
            //WAITING OR PLAY PATH
            if(get_gobot_status.response.status==10 || get_gobot_status.response.status==5){
                std::vector<std::string> command({"d"});
                std::string commandStr = command.at(0) + sep;
                sendCommand("",command,commandStr);
            }
        break;

        default:

        break;
    }
}

/*********************************** SERVICES ***********************************/
bool pausePathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"d"});
    std::string commandStr = command.at(0) + sep;
    checkCommand(commandStr.at(0));
    sendCommand("",command,commandStr);

    return true;
}

bool playPathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"j"});
    std::string commandStr = command.at(0) + sep;
    checkCommand(commandStr.at(0));
    sendCommand("",command,commandStr);
    
    return true;
}

bool stopPathSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"l"});
    std::string commandStr = command.at(0) + sep;
    checkCommand(commandStr.at(0));
    sendCommand("",command,commandStr);

    return true;
}

bool goDockSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::vector<std::string> command({"o"});
    std::string commandStr = command.at(0) + sep;
    checkCommand(commandStr.at(0));
    sendCommand("",command,commandStr);
    return true;
}

bool setDockStatus(gobot_msg_srv::SetDockStatus::Request &req, gobot_msg_srv::SetDockStatus::Response &res){
    ROS_INFO("(Command system) setDockStatus service called %d", req.status);
    dockStatus = req.status;
    if(dockStatus == 1 || dockStatus == 2)
        low_battery = false;

    return true;
}

bool getDockStatus(gobot_msg_srv::GetDockStatus::Request &req, gobot_msg_srv::GetDockStatus::Response &res){
    //ROS_INFO("(Command system) getDockStatus service called");
    res.status = dockStatus;

    return true;
}

bool lowBatterySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    getGobotStatusSrv.call(get_gobot_status);
    //robot is doing the path
    if(get_gobot_status.response.status==10 || get_gobot_status.response.status==5){
        std_srvs::Empty arg;
        ros::service::call("/gobot_function/goDockAfterPath", arg);
        ROS_WARN("(Command system) The battery is low, the robot will go back when it stops being busy with its path");
    }
    else if(get_gobot_status.response.status==15){
        ROS_WARN("(Command system) The robot is trying to dock");
    }
    else{
        ROS_INFO("(Command system) Sending the robot home");
        std::vector<std::string> command({"o"});
        std::string commandStr = command.at(0) + sep;
        checkCommand(commandStr.at(0));
        sendCommand("",command,commandStr);
    }

    return true;
}



/*********************************** COMMUNICATION FUNCTIONS ***********************************/

void sendConnectionData(boost::shared_ptr<tcp::socket> sock){
    ros::NodeHandle n;
    /// Send a message to the app to tell we are connected
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
        //ROS_INFO("(Command system) set homefile to %s", homeFile.c_str());
        std::ifstream ifs(homeFile, std::ifstream::in);
        
        if(ifs){
            ifs >> homeX >> homeY >> x_angle >> y_angle >> z_angle >> w_angle;

            tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(x_angle , y_angle , z_angle, w_angle));

            matrix.getRPY(roll, pitch, yaw);
            homeOri = -(yaw*180/3.14159) - 90;//-(orientation+90)*3.14159/180);
            //ROS_INFO("(Command system) rotation %f", homeOri);

            //ROS_INFO("(Command system) Home : [%s, %s, %f]", homeX.c_str(), homeY.c_str(), homeOri);
            dockStatus = 0;
            ifs.close();
        }
    }

    /// we send the path along with the time of the last modification of its file
    std::string path("");
    std::string pathFile;
    if(n.hasParam("path_file")){
        n.getParam("path_file", pathFile);
        //ROS_INFO("(Command system) set path file to %s", pathFile.c_str());
        std::ifstream ifPath(pathFile, std::ifstream::in);
        
        if(ifPath){
            std::string line("");
            //ROS_INFO("(Command system) Line path");
            while(getline(ifPath, line))
                path += line + sep;
            ifPath.close();
        }
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
        mapId = "{00000000-0000-0000-0000-000000000000}";
    if(mapDate.empty())
        mapDate = "1970-05-21-00-00-00";
    if(homeX.empty())
        homeX = "-150";
    if(homeY.empty())
        homeY = "-150";

     getGobotStatusSrv.call(get_gobot_status);

    std::string scan = (scanning) ? "1" : "0";
    std::string looping_str = (looping) ? "1" : "0";
    std::string following_path_str = (get_gobot_status.response.status==10 || get_gobot_status.response.status==5) ? "1" : "0";

    sendMessageToSock(sock, std::string("Connected" + sep + mapId + sep + mapDate + sep + homeX + sep + homeY + sep + std::to_string(homeOri) + sep + scan + sep + laserStr + sep + following_path_str + sep + looping_str + sep + path));
}

bool sendMessageToSock(boost::shared_ptr<tcp::socket> sock, const std::string message){
    std::string ip = sock->remote_endpoint().address().to_string();
    ROS_INFO("(Command system) Sending message to %s : %s", ip.c_str(), message.c_str());

    try {
        socketsMutex.lock();
        /// We send the result of the command to the given socket
        boost::asio::write(*sock, boost::asio::buffer(message, message.length()));

        ROS_INFO("(Command system) Message sent succesfully");
        socketsMutex.unlock();
        return true;
    } catch (std::exception& e) {
        ROS_ERROR("(Command system) Message not sent : %s", e.what());
        socketsMutex.unlock();
        return false;
    }
}

bool sendMessageToAll(const std::string message){
    //ROS_INFO("(Command system) Sending message : %s", message.c_str());

    try {
        socketsMutex.lock();
        /// We send the result of the command to every Qt app
        for(auto const &elem : sockets)
            boost::asio::write(*(elem.second), boost::asio::buffer(message, message.length()));

        ROS_INFO("(Command system) Message sent succesfully");
        socketsMutex.unlock();
        return true;
    } catch (std::exception& e) {
        ROS_ERROR("(Command system) Message not sent : %s", e.what());
        socketsMutex.unlock();
        return false;
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
        while(ros::ok() && sockets.count(ip)) {
            char data[max_length];

            boost::system::error_code error;
            /// We wait to receive some data
            size_t length = sock->read_some(boost::asio::buffer(data), error);
            if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
                ROS_WARN("(Command system) Connection closed %s", ip.c_str());
                disconnect(ip);
                return;
            } else if (error) {
                disconnect(ip);
                throw boost::system::system_error(error); // Some other error.
            }


            for(int i = 0; i < length; i++){
                //Null
                if(static_cast<int>(data[i]) != 0){
                    //ETB
                    if(static_cast<int>(data[i]) == 23){
                        ROS_INFO("(Command system) Read command complete %s", ip.c_str());
                        finishedCmd = 1;
                        i = length;
                    } else
                        commandStr += data[i];
                }
            }

            if(commandStr.length() > 0){

                /// Split the command from a str to a vector of str
                ///cmd+sep+param1+sep+param2+sep+ETB+sep
                split(commandStr, sep_c, std::back_inserter(command));

                if(finishedCmd){
                    checkCommand(commandStr.at(0));
                    sendCommand(ip,command,commandStr);

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
        ROS_ERROR("(Command system) Exception in thread, ip : %s => %s ", ip.c_str(), e.what());
    }
    ROS_WARN("(Command system) Done with this session %s", ip.c_str());
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
        if(!scanning || scanningIp.compare(ip) == 0){
            ROS_INFO("(Command system) Command socket connected to %s", ip.c_str());
            socketsMutex.lock();
            if(!sockets.count(ip))
                sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
            else
                ROS_ERROR("(Command system) the ip %s is already connected, this should not happen", ip.c_str());
            socketsMutex.unlock();

            /// Launch the session thread which will communicate with the server
            //tx??//one thread for one command (even from the same IP?)
            std::thread(session, sock).detach();
        } else 
            ROS_WARN("(Command system) The ip %s tried to connect to the robot while already scanning with ip %s", ip.c_str(), scanningIp.c_str());
    }
}

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    disconnect(msg->data);
}

void disconnect(const std::string ip){
    ROS_WARN("(Command system) The ip %s just disconnected", ip.c_str());

    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(ip)){
        sockets.at(ip)->close();
        sockets.erase(ip);
    }
    socketsMutex.unlock();
}

/*********************************** MAIN ***********************************/

int main(int argc, char* argv[]){

    try{
        ros::init(argc, argv, "command_system");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);

        go_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        teleop_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        ros::ServiceServer setDockStatusSrv = n.advertiseService("/gobot_status/setDockStatus", setDockStatus);
        ros::ServiceServer getDockStatusSrv = n.advertiseService("/gobot_status/getDockStatus", getDockStatus);

        ros::ServiceServer lowBatterySrv = n.advertiseService("/gobot_command/lowBattery", lowBatterySrvCallback);
        ros::ServiceServer goDockSrv = n.advertiseService("/gobot_command/goDock", goDockSrvCallback);
        ros::ServiceServer pausePathSrv = n.advertiseService("/gobot_command/pause_path", pausePathSrvCallback);
        ros::ServiceServer playPathSrc = n.advertiseService("/gobot_command/play_path", playPathSrvCallback);
        ros::ServiceServer stopPathSrc = n.advertiseService("/gobot_command/stop_path", stopPathSrvCallback);

        getGobotStatusSrv = n.serviceClient<gobot_msg_srv::GetGobotStatus>("/gobot_status/get_gobot_status");
        setGobotStatusSrv = n.serviceClient<gobot_msg_srv::SetGobotStatus>("/gobot_status/set_gobot_status");

        n.param<bool>("simulation", simulation, false);
        ROS_INFO("(Command system) simulation : %d", simulation);

        std::thread t(server);

        ros::spin();
        
    } catch (std::exception& e) {
        ROS_ERROR("(Command system) Exception : %s", e.what());
    }

    return 0;
}
