#include "gobot_software/tcp_read_map.hpp"

#define NEW_MAP_PORT 5601

using boost::asio::ip::tcp;

const int max_length = 1024;
bool simulation = false;

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

ros::Publisher map_pub;

std_srvs::Empty empty_srv;

robot_class::SetRobot SetRobot;
robot_class::GetRobot GetRobot;
int robot_status_;
std::string status_text_;

void session(boost::shared_ptr<tcp::socket> sock){

    std::string ip = sock->remote_endpoint().address().to_string();
    //~ROS_INFO("(MAP_READ) session launched %s", ip.c_str());

    ros::NodeHandle n;
    int gotMapData(0);
    std::string mapId("");
    std::string mapMetadata("");
    std::string mapDate("");
    std::vector<uint8_t> map;

    while(ros::ok() && sockets.count(ip)){
        
        // buffer in which we store the bytes we read on the socket
        char data[max_length];

        boost::system::error_code error;
        size_t length = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            //~ROS_INFO("(MAP_READ) Connection closed");
            return;
        } 
        else if (error) {
            throw boost::system::system_error(error); // Some other error.
            return;
        }
        //ROS_INFO("(MAP_READ) Received Map Data Length: %zu",length);
        // Parse the data as we are supposed to receive : "mapId ; mapDate ; metadata ; map"
        for(int i = 0; i < length; i++){
            if(data[i] == ';' && gotMapData <= 2){
                /// The first ; means we got the mapId
                /// the second means we got the metadata
                /// the third means we got the map date
                gotMapData++;
            } 
            else {
                if(gotMapData > 2)
                    map.push_back((uint8_t) data[i]);
                else if(gotMapData == 0)
                    mapId += data[i];
                else if(gotMapData == 1)
                    mapDate += data[i];
                else 
                    mapMetadata += data[i];
            }
        }

        // when the last 5 bytes are 254 we know we have received a complete map
        if(map.size() > 4 && static_cast<int>(map.at(map.size()-5)) == 254 && 
           static_cast<int>(map.at(map.size()-4)) == 254 && static_cast<int>(map.at(map.size()-3)) == 254 && 
           static_cast<int>(map.at(map.size()-2)) == 254 && static_cast<int>(map.at(map.size()-1)) == 254){

            std::string mapType = mapId.substr(0,4);
            ROS_INFO("(MAP_READ) Map Type: %s", mapType.c_str());
            if(mapType == "EDIT" || mapType == "IMPT" || mapType == "SCAN"){
                mapId = mapId.substr(4);
            }
 

            /// We check if we already have a map with the same id
            std::string mapIdFile;
            n.getParam("map_id_file", mapIdFile);
            //ROS_INFO("(MAP_READ) got map id file : %s", mapIdFile.c_str());

            std::string mapIdFromFile("");
            std::ifstream ifMap(mapIdFile, std::ifstream::in);
            if(ifMap){
                getline(ifMap, mapIdFromFile);
                ifMap.close();
            }

            /// If we have a different map, we replace it
            if(mapIdFromFile.compare(mapId) != 0){
                
                //stop the robot for reading new map
                int move_status = SetRobot.stopRobotMoving();
                if(move_status != 0){
                    ROS_INFO("(MAP_READ) Stop robot motion, current status: %d.", move_status);
                }

                /// Save the id of the new map
                std::ofstream ofs(mapIdFile, std::ofstream::out | std::ofstream::trunc);
                if(ofs){
                    ofs << mapId << std::endl << mapDate << std::endl;
                    ofs.close();
                    ROS_INFO("(MAP_READ) Update map id %s with date %s to mapIdFile %s", mapId.c_str(), mapDate.c_str(),mapIdFile.c_str());

                    /// Set the medatada of the new map
                    int width(0);
                    int height(0);
                    double resolution(0.0f);
                    double initPosX(0.0f);
                    double initPosY(0.0f);
                    double orientation(0.0f);

                    std::istringstream iss(mapMetadata);
                    iss >> width >> height >> resolution >> initPosX >> initPosY >> orientation;
                    ROS_INFO("(MAP_READ) Map metadata after split : %d %d %f %f %f %f", width, height, resolution, initPosX, initPosY, orientation);

                    /// We remove the 5 last bytes as they are only there to identify the end of the map
                    map.erase(map.end() - 5, map.end());
                    ROS_INFO("(MAP_READ) Size of the map received : %lu", map.size());
                    
                    /// Update the config file
                    std::string mapConfig;
                    n.getParam("map_config_used", mapConfig);
                    ofs.open(mapConfig, std::ofstream::out | std::ofstream::trunc);
                    if(ofs.is_open()){
                        std::string resolutionStr = "resolution: " + std::to_string(resolution);
                        std::string originStr = "origin: [" + std::to_string(initPosX) + ", " + std::to_string(initPosY) + ", 0.00]";
                        ofs << "image: used_map.pgm" << std::endl << resolutionStr << std::endl << originStr << std::endl << "negate: 0" << std::endl << "occupied_thresh: 0.65" << std::endl << "free_thresh: 0.196";
                        ofs.close();
                        //ROS_INFO("(MAP_READ) New map config file created in %s", mapConfig.c_str()); 
                    }

                    /// We save the file in a the pgm file for amcl navigation
                    std::string mapFile;
                    n.getParam("map_image_used", mapFile);
                    ofs.open(mapFile, std::ofstream::out | std::ofstream::trunc);
                    if(ofs.is_open()){
                        /// pgm file header
                        ofs << "P5" << std::endl << width << " " << height << std::endl << "255" << std::endl;
                        /// writes every single pixel to the pgm file
                        for(int i = 0; i < map.size(); i+=5){
                            uint8_t color = static_cast<uint8_t> (map.at(i));
                            uint32_t count2 = static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+1)) << 24) + static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+2)) << 16)
                                            + static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+3)) << 8) + static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+4)));
            
                            for(int j = 0; j < count2; j++)
                                ofs << color;
                        }
                        ofs << std::endl;
                        ofs.close();
                        //ROS_INFO("(MAP_READ) New map pgm file created in %s", mapFile.c_str());
                    }
                    
                    //disconnect all other users to receive new map, so no need feedback to TCP
                    ROS_INFO("(MAP_READ) Disconnect other uses to update them new map.");
                    ros::service::call("/gobot_software/disconnet_servers",empty_srv);
                    
                    if(mapType != "EDIT"){
                        //#### delete old robot data ####
                        if(mapType != "IMPT" && mapType != "SCAN"){
                            /// We delete the old home
                            SetRobot.setHome("0","0","0","0","0","1");
                        }

                        /// We detele the loop
                        SetRobot.setLoop(0);
                        /// We delete the old path
                        SetRobot.clearPath();
                        /// We delete the old path stage
                        SetRobot.setStage(0);
                        //#### delete old robot data ####
                    }

                    if(mapType == "EDIT"){
                        SetRobot.reloadMap();
                    }
                    else if(mapType == "IMPT"){
                        SetRobot.reloadMap();
                        gobot_msg_srv::IsCharging isCharging;
                        ros::service::call("/gobot_status/charging_status", isCharging);
                        if(isCharging.response.isCharging){
                            gobot_msg_srv::GetStringArray get_home;
                            ros::service::call("/gobot_status/get_home",get_home);
                            SetRobot.setInitialpose(std::stod(get_home.response.data[0]),std::stod(get_home.response.data[1]),
                                                    std::stod(get_home.response.data[2]),std::stod(get_home.response.data[3]),
                                                    std::stod(get_home.response.data[4]),std::stod(get_home.response.data[5]));
                        }
                    }
                    else{
                        ROS_INFO("(MAP_READ) We relaunched gobot_navigation");
                        /// Kill gobot move so that we'll restart it with the new map
                        std::string cmd = SetRobot.killList();
                        system(cmd.c_str());
                        ROS_INFO("(MAP_READ) We killed gobot_navigation");
                        /// Relaunch gobot_navigation
                        SetRobot.runNavi(simulation);

                        ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(10.0));
                        gobot_msg_srv::GetStringArray get_home;
                        ros::service::call("/gobot_status/get_home",get_home);
                        SetRobot.setInitialpose(std::stod(get_home.response.data[0]),std::stod(get_home.response.data[1]),
                                                std::stod(get_home.response.data[2]),std::stod(get_home.response.data[3]),
                                                std::stod(get_home.response.data[4]),std::stod(get_home.response.data[5]));
                    }
                } 
                else {
                    ROS_INFO("(MAP_READ) Map id could not be updated : %s with date %s", mapId.c_str(), mapDate.c_str());
                }
            } 
            else{
                ROS_INFO("(MAP_READ) SAME IDS ");
                std::string message = "done 0";
                //feedback to TCP if receiving same map
                boost::asio::write(*sock, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), error);
            }

            //close this session
            return;
        }
    }
}

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), NEW_MAP_PORT));
    while(ros::ok()) {

        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));

        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        //~ROS_INFO("(MAP_READ) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();

        if(sockets.find(ip)==sockets.end()){ //not find the ip
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
        }
        else{   //ip exists in the list
            sockets.find(ip)->second->close();
            sockets.find(ip)->second = sock;
        }
        socketsMutex.unlock();

        /// Launch the session thread which wait for a new map
        std::thread(session, sock).detach();
    }
}

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(msg->data)){
        sockets.at(msg->data)->close();
        sockets.erase(msg->data);
        //~ROS_WARN("(MAP_READ) The ip %s just disconnected", msg->data.c_str());
    }
    socketsMutex.unlock();
}

/*********************************** SHUT DOWN ***********************************/
void mySigintHandler(int sig){ 

    ros::shutdown();
}

/*********************************** MAIN ***********************************/

int main(int argc, char **argv){

    ros::init(argc, argv, "tcp_read_map");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    
    SetRobot.initialize();
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    //Startup end

    n.getParam("simulation", simulation);
    ROS_INFO("(MAP_READ) simulation : %d", simulation);

    /// Subscribe to know when we disconnected from the server
    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1, serverDisconnected);

    /// Advertise that we are going to publish to /map
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 100);

    ROS_INFO("(MAP_READ) Ready to be launched.");

    std::thread t(server);

    ros::spin();

    return 0;
}
