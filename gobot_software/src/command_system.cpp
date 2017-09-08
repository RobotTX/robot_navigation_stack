#include "command_system.hpp"

const int max_length = 1024;

bool waiting = false;
bool connected = false;
bool scanning = false;
bool recovering = false;
bool laserActivated = false;
bool simulation = false;

ros::ServiceClient startRobotPosClient;
ros::ServiceClient stopRobotPosClient;

ros::ServiceClient startMapClient;
ros::ServiceClient sendOnceMapClient;
ros::ServiceClient sendAutoMapClient;
ros::ServiceClient stopAutoMapClient;
ros::ServiceClient stopMapClient;

ros::ServiceClient playPathClient;
ros::ServiceClient pausePathClient;
ros::ServiceClient stopPathClient;

ros::ServiceClient startLaserClient;
ros::ServiceClient sendLaserClient;
ros::ServiceClient stopSendLaserClient;
ros::ServiceClient stopLaserClient;

ros::ServiceClient recoverPositionClient;
ros::ServiceClient stopRecoveringPositionClient;
ros::ServiceClient checkLocalizationClient;
ros::ServiceClient stopCheckingLocalizationClient;

// to get the local map to be send to recover the robot's position
ros::ServiceClient sendLocalMapClient;
ros::ServiceClient stopSendingLocalMapClient;

ros::Publisher go_pub;
ros::Publisher teleop_pub;

int robot_pos_port = 4001;
int map_port = 4002;
int laser_port = 4003;
int recovered_position_port = 4004;
int dockStatus = -2;

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

bool execCommand(ros::NodeHandle n, const std::vector<std::string> command){

    std::string commandStr = command.at(0);
    bool status(false);
    switch (commandStr.at(0)) {

        /// Command for changing the name of the robot
        case 'a':
            // first param = command, second param = nom 
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
                    status = true;
                }
                
            } else 
                ROS_INFO("(Command system) Name missing");
        break;

        /// Command for changing the wifi of the robot
        case 'b':
            // first param = b, second param = new ssid, third param = password
            if(command.size() == 3){
                ROS_INFO("(Command system) New wifi : %s", command.at(1).c_str());
                ROS_INFO("(Command system) New wifi password : %s", command.at(2).c_str());
                std::string cmd = "sudo bash ~/computer_software/change_wifi.sh \"" + command.at(1) + "\" \""+ command.at(2) + "\"";
                ROS_INFO("(Command system) Cmd : %s", cmd.c_str());

                system(cmd.c_str());
                status = true;
            } else 
                ROS_INFO("(Command system) Parameter missing");
        break;
        
        /// Command for the robot to move to a point
        case 'c':
            // first param == c, second param = goal pos x coordinate, third param = goal pos y coordinate
            ROS_INFO("(Command system) Gobot go to point");
            if(command.size() == 3){
                double posX = std::stof(command.at(1));
                double posY = std::stof(command.at(2));

                /// Before setting a new goal, we stop any teleoperation command
                stopTwist();

                /// Send a goal
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

                status = true;
                dockStatus = 0;
            } else 
                ROS_INFO("(Command system) Parameter missing");
        
        break;

        /// Command for the robot to pause the path
        case 'd':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Pausing the path");
                std_srvs::Empty arg;
                if(ros::service::call("pause_path", arg)){
                    ROS_INFO("(Command system) Pause path service called with success");
                    status = true;
                } else
                    ROS_INFO("(Command system) Pause path service call failed");
            }
        break;

        /// Command for the robot to play the ongoing scan
        case 'e':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Gobot play the ongoing scan");
                status = sendAutoMap();
            }
        break;

        /// Command for the robot to pause the ongoing scan
        case 'f':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Gobot pause the ongoing scan");
                stopExploration();
                status = stopAutoMap();
            }
        break;

        case 'g':
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
                    status = sendAutoMap();
                else
                    ROS_INFO("(Command system) Could not call the service /move_base_controller/startExploration");


            } else 
                ROS_INFO("(Command system) Parameter missing");
        break;

        /// Command for the robot to receive the ports needed for the map and robot pos services
        case 'h':
            // first param is h, 2nd is port for robot position, 3rd for map, 4th for laser
            if(command.size() == 4){
                robot_pos_port = std::stoi(command.at(1));
                map_port = std::stoi(command.at(2));
                laser_port = std::stoi(command.at(3));
                ROS_INFO("(Command system) Gobot here are the ports %d, %d, %d", robot_pos_port, map_port, laser_port);
                startRobotPos();
                startMap();
                startLaserData(laserActivated);
                //connectToParticleCloudNode();
                status = true;
            } else
                ROS_INFO("(Command system) Parameter missing");
        break;


        /// Command for the robot to save a new path
        case 'i':
            // first param is i, then the path name, then quadriplets of parameters to represent path points (path point name, posX, posY, waiting time) 
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
                                ROS_INFO("Stop path service call failed");

                            status = true;
                        } else
                            ROS_INFO("Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played");
                    } else 
                        ROS_INFO("sorry could not open the file %s", pathFile.c_str());
                }
                
            } else 
                ROS_INFO("(Command system) Parameter missing %lu %lu", command.size(), command.size()%4);
        break;

        /// Command for the robot to play the saved path
        case 'j':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Playing the path");
                std_srvs::Empty arg;
                if(ros::service::call("play_path", arg)){
                    ROS_INFO("(Command system) Play path service called with success");
                    status = true;
                    dockStatus = 0;
                } else
                    ROS_INFO("(Command system) Play path service call failed");
            }
        break;

        /// Command for the robot to delete the saved path
        case 'k':
            /*if(command.size() == 1) {
                {
                    ROS_INFO("(Command system) Deleting the path");
                    std::ofstream ofs;
                    ofs.open(path_computer_software + "Robot_Infos/path.txt", std::ofstream::out | std::ofstream::trunc);
                    ofs.close();
                }
                status = true;
            }*/
        break;

        /// Command to stop the robot while following its path
        case 'l':
            if(command.size() == 1) {
                
                ROS_INFO("(Command system) Stopping the path");
                std_srvs::Empty arg;
                if(ros::service::call("stop_path", arg)){
                    ROS_INFO("(Command system) Stop path service called with success");
                    status = true;
                } else
                    ROS_INFO("(Command system) Stop path service call failed");
            }
        break;

        // command to stop the robot and then delete its path
        case 'm':
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
                        status = true;
                    }
                } else
                    ROS_INFO("(Command system) Stop path service call failed");
            }
        break;

        // command to save the home of the robot
        case 'n':
            // param 1 is n, 2nd is the home x coordinate, 3rd is the home y coordinate, 4th is the orientation of the home
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
                        status = true;
                        dockStatus = 0;

                    } else
                        ROS_INFO("(Command system) sorry could not open the file %s", homeFile.c_str());
                }
                
            } else
                ROS_INFO("(Command system) Not enough arguments, received %lu arguments, 4 arguments expected", command.size());
        break;

        // command to send the robot home
        case 'o':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Sending the robot home");
                std_srvs::Empty arg;

                if(ros::service::call("startDocking", arg)){
                    ROS_INFO("(Command system) Go home service called with success");
                    status = true;
                    dockStatus = 3;
                } else
                    ROS_INFO("(Command system) Stop path service call failed");
            }
        break;

        // command so that the robot stops on its way home
        case 'p':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Stopping the robot on its way home");
                std_srvs::Empty arg;

                if(ros::service::call("stopDocking", arg)){
                    ROS_INFO("(Command system) Stop going home service called with success");
                    status = true;
                    dockStatus = 0;
                } else
                    ROS_INFO("(Command system) Stop going home service call failed");
            }
        break;

        case 'q':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Gobot sends laser data");
                status = sendLaserData();

                if(status){
                    std::string laserFile;
                    if(n.hasParam("laser_file")){
                        n.getParam("laser_file", laserFile);
                        ROS_INFO("(Command system) set laser file to %s", laserFile.c_str());
                        std::ofstream ofs;
                        ofs.open(laserFile, std::ofstream::out | std::ofstream::trunc);
                        ofs << "1";
                        ofs.close();
                    }
                }
            }
        break;

        case 'r':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Gobot stops sending laser data");
                status = stopSendLaserData();

                if(status){
                    std::string laserFile;
                    if(n.hasParam("laser_file")){
                        n.getParam("laser_file", laserFile);
                        ROS_INFO("(Command system) set laser file to %s", laserFile.c_str());
                        std::ofstream ofs;
                        ofs.open(laserFile, std::ofstream::out | std::ofstream::trunc);
                        ofs << "0";
                        ofs.close();
                    }
                }
            }
        break;

        /// Command for the robot to send its map once
        case 's':
            // first param is s, second is who -> which widget requires it
            ROS_INFO("(Command system) Gobot send the map once");
            if(command.size() == 2)
                status = sendOnceMap(std::stoi(command.at(1)));
             else 
                ROS_INFO("(Command system) Not enough arguments, received %lu arguments, 2 arguments expected", command.size()); 
        break;

        /// Command for the robot to start a scan from the beggining
        case 't':
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

                status = sendAutoMap();
            }
        break;

        /// Command for the robot to stop a scan
        case 'u':
            if(command.size() == 2) {
                ROS_INFO("(Command system) Gobot stops the scan of the new map");
                scanning = false;

                if(std::stoi(command.at(1)) == 1){
                    stopExploration();

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

                status = stopAutoMap();
            }
        break;

        /// command to recover the robot's position
        case 'v':
        /*
            if(command.size() == 1) {
                ROS_INFO("(Command system) Gobot starts recovering its position");
                recovering = true;
                if(sendLocalMap())
                    status = recoverPosition();
                /*
                if(startSendingParticleCloud())
                    return recoverPosition();
                else
                    ROS_INFO("(Command system) Could not get the local map");
                    
            }
        */
        break;

        /// command to pause during the recovery of a robot's position
        case 'w':
        /*
            if(command.size() == 1) {
                ROS_INFO("(Command system) Gobot pauses during the recovery of its position");
                status = stopRecoveringPosition();
            }
        */
        /// command to stop recovering the robot's position
        case 'x':/*
            if(command.size() == 1){
                ROS_INFO("(Command system) Gobot stops during the recovery of its position");
                recovering = false;
                status = stopRecoveringPosition();
            }
        */
        break;

        case 'y':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Gobot resumes recovering its position");
                recovering = true;
                if(sendLocalMap())
                    status = resumeRecoveryPosition();
                else
                    ROS_INFO("(Command system) Could not get the local map");
            }
        break;

        case 'z':
            /// TODO finish this shit
            if(command.size() == 1){
                ROS_INFO("(Command system) Gobot restarts its packages");
                recovering = false;
                scanning = false;
                system(("sh ~/computer_software/restart_packages.sh"));
                sleep(10);
                system(("sh ~/computer_software/roslaunch.sh"));
                /*
                // to kill gobot move and gobot software using the fact that those two nodes are required (see launch files)
                std::string cmd = "rosnode kill /move_base";
                system(cmd.c_str());
                cmd = "rosnode kill /robot_pos_transfer &";
                system(cmd.c_str());
                sleep(5);
                /// Relaunch gobot_navigation
                
                cmd = "roslaunch gobot_software slam.launch &";
                system(cmd.c_str());
                cmd = "roslaunch gobot_navigation slam.launch &";
                system(cmd.c_str());
                */
                status = true;
            }
        break;

        /// Command for the robot to start to explore the map automatically
        case ',':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Start to explore");
                /// 0 : the robot doesn't go back to its starting point at the end of the scan
                /// 1 : robot goes back to its starting point which is its charging station
                /// 2 : robot goes back to its starting point which is not a charging station
                hector_exploration_node::Exploration arg;
                arg.request.backToStartWhenFinished = 2;
                if(ros::service::call("/move_base_controller/startExploration", arg))
                    status = true;
                else
                    ROS_INFO("(Command system) Could not call the service /move_base_controller/startExploration");
            }
        break;

        /// Command for the robot to stop to explore the map automatically
        case '.':
            if(command.size() == 1) {
                ROS_INFO("(Command system) Stop to explore");
                status = stopExploration();
            }
        break;

        /// Command for the robot to loop the path
        case '/':
            if(command.size() == 2) {
                ROS_INFO("(Command system) Loop the path %s", command.at(1).c_str());
                std_srvs::Empty arg;
                if(std::stoi(command.at(1)) == 0){
                    if(ros::service::call("/stopLoopPath", arg))
                        status = true;
                    else
                        ROS_INFO("(Command system) Could not call the service /stopLoopPath");
                } else {
                    if(ros::service::call("/startLoopPath", arg))
                        status = true;
                    else
                        ROS_INFO("(Command system) Could not call the service /startLoopPath");
                }
            }
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

bool stopExploration(void){

    std_srvs::Empty arg;
    if(!ros::service::call("/move_base_controller/stopExploration", arg)){
        ROS_INFO("(Command system) Could not call the service /move_base_controller/stopExploration");
        return false;
    }

    return true;
}

bool resumeRecoveryPosition(void){
    ROS_INFO("(Command system) Launching the service to resume the recovery of the robot's position");
    std_srvs::Empty srv;

    if (recoverPositionClient.call(srv)) {
        ROS_INFO("(Command system) resume recover_position started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to resume recover_position");
        return false;
    }
}

bool stopRecoveringPosition(void){
    ROS_INFO("Stop recovering position called");
    std_srvs::Empty srv;
    stopCheckingLocalizationClient.call(srv);
    if(stopRecoveringPositionClient.call(srv)){
        if(stopSendingLocalMapClient.call(srv)){
            ROS_INFO("(Command system) Stopped sending the local map");
            return true;
        }
        else {
            ROS_INFO("(Command system) Could not stop sending the local map");
            return false;
        }
    }

    else { 
        ROS_INFO("(Command system) Could not stop the recover position service");
        return false;
    }
    /*
    ROS_INFO("(Command system) Stop recovering position called");
    std_srvs::Empty srv;
    stopCheckingLocalizationClient.call(srv);
    if(stopRecoveringPositionClient.call(srv)){
        if(stopSendingLocalMapClient.call(srv)){
            ROS_INFO("(Command system) Stopped sending the local map");
            return true;
        }
        else {
            ROS_INFO("(Command system) Could not stop sending the local map");
            return false;
        }
    }

    else { 
        ROS_INFO("(Command system) Could not stop the recover position service ");
        return false;
    }
    */
}

/**

bool recoverPosition(void){
ROS_INFO("(Command system) Launching the service to recover the robot's position");
    std_srvs::Empty srv;

    if (initializeParticleCloudClient.call(srv) && recoverPositionClient.call(srv)) {
        ROS_INFO("(Command system) recover_position service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service recover_position");
        return false;
    }

    ROS_INFO("(Command system) Launching the service to recover the robot's position");
    std_srvs::Empty srv;

    if (recoverPositionClient.call(srv)){
        ROS_INFO("(Command system) succesfully called recover_position service, now trying to call checkLocalization service");
        if(checkLocalizationClient.call(srv)){
            ROS_INFO("(Command system) checkLocalization succesfully launched, recovery can start");
            return true;
        } 
        else {
            ROS_INFO("(Command system) recover_position service could not start");
            return false;
        }
    } else {
        ROS_ERROR("(Command system) Failed to call service recover_position");
        return false;
    }

}
*/
void stopTwist(void){
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    teleop_pub.publish(twist);

    ros::spinOnce();
}

void startRobotPos(void){
    ROS_INFO("(Command system) Launching the service to get the robot position");

    gobot_software::Port srv;
    srv.request.port = robot_pos_port;

    if (startRobotPosClient.call(srv)) 
        ROS_INFO("(Command system) start_robot_pos_sender service started");
    else 
        ROS_ERROR("(Command system) Failed to call service start_robot_pos_sender");
}

void stopRobotPos(void){
    std_srvs::Empty srv;

    if (stopRobotPosClient.call(srv)) 
        ROS_INFO("(Command system) stop_robot_pos_sender service started");
    else 
        ROS_ERROR("(Command system) Failed to call service stop_robot_pos_sender");
}


bool startMap(void){
    ROS_INFO("(Command system) Launching the service to open the map socket");

    gobot_software::Port srv;
    srv.request.port = map_port;

    if (startMapClient.call(srv)) {
        ROS_INFO("(Command system) start_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service start_map_sender");
        return false;
    }
}

// who:
// 0 : scan 
// 1 : application requesting at connection time
// 2 : to merge
// 3 : recovering position
bool sendOnceMap(const int who){
    ROS_INFO("(Command system) Launching the service to get the map once");

    gobot_software::Port srv;
    srv.request.port = who;

    if (sendOnceMapClient.call(srv)) {
        ROS_INFO("(Command system) send_once_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service send_once_map_sender");
        return false;
    }
}

bool sendAutoMap(void){
    ROS_INFO("(Command system) Launching the service to get the map auto");

    std_srvs::Empty srv;

    if (sendAutoMapClient.call(srv)) {
        ROS_INFO("(Command system) send_auto_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service send_auto_map_sender");
        return false;
    }
}

bool sendLocalMap(void){
    ROS_INFO("(Command system) Launching the service to get the map auto");

    std_srvs::Empty srv;

    if (sendLocalMapClient.call(srv)) {
        ROS_INFO("(Command system) send_local_map service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service send_local_map");
        return false;
    }
}

bool stopAutoMap(void){
    ROS_INFO("(Command system) Launching the service to stop the map auto");

    std_srvs::Empty srv;

    if (stopAutoMapClient.call(srv)) {
        ROS_INFO("(Command system) stop_auto_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service stop_auto_map_sender");
        return false;
    }
}

bool stopMap(void){

    std_srvs::Empty srv;
    if (stopMapClient.call(srv)) {
        ROS_INFO("(Command system) stop_map_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service stop_map_sender");
        return false;
    }
}

bool startLaserData(const bool startLaser){
    ROS_INFO("(Command system) Launching the service which will send the lasers's data using port %d", laser_port);
    gobot_software::PortLaser srv;
    srv.request.port = laser_port;
    srv.request.startLaser = startLaser;

    if(startLaserClient.call(srv)) {
        ROS_INFO("(Command system) start_laser_data_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service start_laser_data_sender");
        return false;
    }
}

bool sendLaserData(void){
    ROS_INFO("(Command system) Launching the service to get the laser data");
    std_srvs::Empty srv;

    if(sendLaserClient.call(srv)) {
        ROS_INFO("(Command system) send_laser_data_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service send_laser_data_sender");
        return false;
    }
}

bool stopSendLaserData(void){
    ROS_INFO("(Command system) Launching the service to stop receiving the laser data");
    std_srvs::Empty srv;

    if(stopSendLaserClient.call(srv)) {
        ROS_INFO("(Command system) stop_send_laser_data_sender service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service stop_send_laser_data_sender");
        return false;
    }
}

bool stopLaserData(void){
    std_srvs::Empty srv;
    if(stopLaserClient.call(srv)){
        ROS_INFO("Command system stop_sending_laser_data started");
        return true;
    } else {
        ROS_ERROR("(Command system) failed to call service stop_sending_laser_data");
        return false;
    }
}
/*
bool connectToParticleCloudNode(void){
    gobot_software::Port srv;
    srv.request.port = particle_cloud_port;

    if (connectToParticleCloudClient.call(srv)) {
        ROS_INFO("(Command system) connect_particle_cloud service started");
        return true;
    } else {
        ROS_ERROR("(Command system) Failed to call service connect_particle_cloud");
        return false;
    }
}
*/

bool setDockStatus(gobot_software::SetDockStatus::Request &req, gobot_software::SetDockStatus::Response &res){
    ROS_INFO("(Command system) setDockStatus service called %d", req.status);
    dockStatus = req.status;

    return true;
}

bool getDockStatus(gobot_software::GetDockStatus::Request &req, gobot_software::GetDockStatus::Response &res){
    //ROS_INFO("(Command system) getDockStatus service called ");
    res.status = dockStatus;

    return true;
}

void getPorts(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n){

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

        execCommand(n, command);
        ROS_INFO("(Command system) GetPorts done");
    }
}

void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n){
    ROS_INFO("(Command system) Waiting for a command");
    try{
        std::vector<std::string> command;
        std::string commandStr = "";
        bool finishedCmd = 0;

        getPorts(sock, n);

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

                    std::string msg = (execCommand(n, command) ? "done" : "failed") + sep + commandStr;
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

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n){
    ROS_INFO("(Command system) Waiting for connection");

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

    sendMessageToPc(sock, "Connected" + sep + mapId + sep + mapDate + sep + homeX + sep + homeY + sep + std::to_string(homeOri) + sep
        + scan + sep + recover + sep + laserStr + sep + path);

    boost::thread t(boost::bind(session, sock, n));
}

void server(const unsigned short port, ros::NodeHandle n){

    boost::shared_ptr<boost::asio::io_service> io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
    io_service->run();

    boost::shared_ptr<tcp::endpoint> m_endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), port));
    boost::shared_ptr<tcp::acceptor> m_acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*io_service, *m_endpoint));

    m_acceptor->set_option(tcp::acceptor::reuse_address(true));

    ros::Rate r(10);
    while(ros::ok()){
        if(!connected && !waiting){
            ROS_INFO("(Command system) Ready to connect");
            boost::thread t(boost::bind(asyncAccept, io_service, m_acceptor, n));

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
        ROS_INFO("(Command system) Robot could not find the application ");
        stopRobotPos();
        stopMap();
        stopLaserData();
        connected = false;
    }
}

int main(int argc, char* argv[]){

    try{
        ros::init(argc, argv, "command_system");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);
        
        startRobotPosClient = n.serviceClient<gobot_software::Port>("start_robot_pos_sender");
        stopRobotPosClient = n.serviceClient<std_srvs::Empty>("stop_robot_pos_sender");
        
        startMapClient = n.serviceClient<gobot_software::Port>("start_map_sender");
        sendOnceMapClient = n.serviceClient<gobot_software::Port>("send_once_map_sender");
        sendAutoMapClient = n.serviceClient<std_srvs::Empty>("send_auto_map_sender");
        stopAutoMapClient = n.serviceClient<std_srvs::Empty>("stop_auto_map_sender");
        stopMapClient = n.serviceClient<std_srvs::Empty>("stop_map_sender");

        playPathClient = n.serviceClient<std_srvs::Empty>("play_path");
        pausePathClient = n.serviceClient<std_srvs::Empty>("pause_path");
        stopPathClient = n.serviceClient<std_srvs::Empty>("stop_path");

        startLaserClient = n.serviceClient<gobot_software::PortLaser>("start_laser_data_sender");
        sendLaserClient = n.serviceClient<std_srvs::Empty>("send_laser_data_sender");
        stopSendLaserClient = n.serviceClient<std_srvs::Empty>("stop_send_laser_data_sender");
        stopLaserClient = n.serviceClient<std_srvs::Empty>("stop_laser_data_sender");

        recoverPositionClient = n.serviceClient<std_srvs::Empty>("recover_position");
        stopRecoveringPositionClient = n.serviceClient<std_srvs::Empty>("stop_recovering_position");
        checkLocalizationClient = n.serviceClient<std_srvs::Empty>("check_localization");
        stopCheckingLocalizationClient = n.serviceClient<std_srvs::Empty>("stop_checking_localization");

        sendLocalMapClient = n.serviceClient<std_srvs::Empty>("send_local_map");
        stopSendingLocalMapClient = n.serviceClient<std_srvs::Empty>("stop_sending_local_map");

        go_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        teleop_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        ros::ServiceServer setDockStatusSrv = n.advertiseService("setDockStatus", setDockStatus);
        ros::ServiceServer getDockStatusSrv = n.advertiseService("getDockStatus", getDockStatus);

        n.param<bool>("simulation", simulation, false);
        ROS_INFO("(New Map) simulation : %d", simulation);

        ros::spinOnce();

        server(CMD_PORT, n);
        
    } catch (std::exception& e) {
        ROS_ERROR("(Command system) Exception : %s", e.what());
    }

    return 0;
}
