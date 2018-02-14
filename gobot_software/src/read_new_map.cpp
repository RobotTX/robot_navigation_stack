#include "gobot_software/read_new_map.hpp"

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
    //~ROS_INFO("(New Map) session launched %s", ip.c_str());

    ros::NodeHandle n;
    int gotMapData(0);
    std::string mapId("");
    std::string mapMetadata("");
    std::string mapDate("");
    std::vector<uint8_t> map;
    std::string message("done 0");

    while(ros::ok() && sockets.count(ip)){
        
        // buffer in which we store the bytes we read on the socket
        char data[max_length];

        boost::system::error_code error;
        size_t length = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            //~ROS_INFO("(New Map) Connection closed");
            return;
        } 
        else if (error) {
            throw boost::system::system_error(error); // Some other error.
            return;
        }

        // Parse the data as we are supposed to receive : "mapId ; mapDate ; metadata ; map"
        for(int i = 0; i < length; i++){
            if(data[i] == ';' && gotMapData <= 2){
                /// The first ; means we got the mapId
                /// the second means we got the metadata
                /// the third means we got the map date
                ROS_INFO("(New Map) ';' found");
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
        if(map.size() > 4 && static_cast<int>(map.at(map.size()-5)) == 254 && static_cast<int>(map.at(map.size()-4)) == 254
             && static_cast<int>(map.at(map.size()-3)) == 254 && static_cast<int>(map.at(map.size()-2)) == 254 && static_cast<int>(map.at(map.size()-1)) == 254){

            std::string mapType = mapId.substr(0,4);
            ROS_INFO("(New Map) Map Type: %s", mapType.c_str());
            if(mapType == "EDIT" || mapType == "IMPT" || mapType == "SCAN"){
                mapId = mapId.substr(4);
            }
 

            /// We check if we already have a map with the same id
            std::string mapIdFile;
            if(n.hasParam("map_id_file")){
                n.getParam("map_id_file", mapIdFile);
                ROS_INFO("(New Map) got map id file : %s", mapIdFile.c_str());
            }
            std::string mapIdFromFile("");
            std::ifstream ifMap(mapIdFile, std::ifstream::in);
            if(ifMap){
                getline(ifMap, mapIdFromFile);
                ifMap.close();
            }

            /// If we have a different map, we replace it
            if(mapIdFromFile.compare(mapId) != 0){
                
                //stop the robot for reading new map
                GetRobot.getStatus(robot_status_);
                if(robot_status_==15){
                    ros::service::call("/gobot_command/stopGoDock",empty_srv);
                    ROS_INFO("(New Map) stop auto docking to read new map.");
                }
                else if(robot_status_==5){
                    ros::service::call("/gobot_command/pause_path",empty_srv);
                    ROS_INFO("(New Map) stop gobot to read new map.");
                }
                
                /// Save the id of the new map
                std::ofstream ofs(mapIdFile, std::ofstream::out | std::ofstream::trunc);
                if(ofs){
                    ofs << mapId << std::endl << mapDate << std::endl;
                    ofs.close();
                    ROS_INFO("(New Map) Update map id %s with date %s to mapIdFile %s", mapId.c_str(), mapDate.c_str(),mapIdFile.c_str());

                    /// Set the medatada of the new map
                    int width(0);
                    int height(0);
                    double resolution(0.0f);
                    double initPosX(0.0f);
                    double initPosY(0.0f);
                    double orientation(0.0f);

                    std::istringstream iss(mapMetadata);
                    iss >> width >> height >> resolution >> initPosX >> initPosY >> orientation;
                    ROS_INFO("(New Map) Map metadata after split : %d %d %f %f %f %f", width, height, resolution, initPosX, initPosY, orientation);

                    /// We remove the 5 last bytes as they are only there to identify the end of the map
                    map.erase(map.end() - 5, map.end());
                    ROS_INFO("(New Map) Size of the map received : %lu", map.size());
                    
                    /// Update the config file
                    std::string mapConfig;
                    if(n.hasParam("map_config_used")){
                        n.getParam("map_config_used", mapConfig);

                        ofs.open(mapConfig, std::ofstream::out | std::ofstream::trunc);
                        if(ofs.is_open()){
                            std::string resolutionStr = "resolution: " + std::to_string(resolution);
                            std::string originStr = "origin: [" + std::to_string(initPosX) + ", " + std::to_string(initPosY) + ", 0.00]";
                            ofs << "image: used_map.pgm" << std::endl << resolutionStr << std::endl << originStr << std::endl << "negate: 0" << std::endl << "occupied_thresh: 0.65" << std::endl << "free_thresh: 0.196";
                            ofs.close();
                            ROS_INFO("(New Map) New map config file created in %s", mapConfig.c_str()); 
                        }
                    }

                    /// We save the file in a the pgm file for amcl navigation
                    std::string mapFile;
                    if(n.hasParam("map_image_used")){
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
                            ROS_INFO("(New Map) New map pgm file created in %s", mapFile.c_str());
                        }

                        /// Kill gobot move so that we'll restart it with the new map
                        std::string cmd = SetRobot.killList();
                        system(cmd.c_str());

                        ROS_INFO("(New Map) We killed gobot_navigation");

                        if(mapType != "EDIT"){
                            //#### delete old robot data ####
                            if(mapType != "IMPT" && mapType != "SCAN"){
                                /// We delete the old home
                                SetRobot.setHome("0","0","0","0","0","1");
                                ROS_INFO("(New Map) Home deleted");
                            }

                            if(mapType == "SCAN"){
                                //Record last known pose in scanned map
                                if(n.hasParam("last_known_position_file")){
                                    std::string lastPoseFile;
                                    n.getParam("last_known_position_file", lastPoseFile);
                                    std::ofstream ofs(lastPoseFile, std::ofstream::out | std::ofstream::trunc);
                                    
                                    gobot_msg_srv::GetStringArray get_home;
                                    ros::service::call("/gobot_status/get_home",get_home);
                                    if(ofs.is_open()){
                                        ofs << get_home.response.data[0] << " " << get_home.response.data[1] << " " << get_home.response.data[2] << " " << get_home.response.data[3] << " " << get_home.response.data[4] << " "<< get_home.response.data[5];
                                        ofs.close();
                                    }
                                    ROS_INFO("(New Map) Robot pose recored in scanned map");
                                }
                            }
                            
                            /// We detele the loop
                            SetRobot.setLoop(0);
                            ROS_INFO("(New Map) Loop deleted");

                            /// We delete the old path
                            SetRobot.clearPath();
                            ROS_INFO("(New Map) Path deleted");

                            /// We delete the old path stage
                            SetRobot.setStage(0);
                            ROS_INFO("(New Map) Path stage deleted");
                            //#### delete old robot data ####
                        }

                        /// Relaunch gobot_navigation
                        SetRobot.runNavi(simulation);
                            
                        ROS_INFO("(New Map) We relaunched gobot_navigation");
                        message = "done 1";

                    } 
                    else {
                        ROS_INFO("(New Map) Could not open the file to create a new pgm file %s", mapFile.c_str());
                        message = "failed";
                    }
                } 
                else {
                    ROS_INFO("(New Map) Map id could not be updated : %s with date %s", mapId.c_str(), mapDate.c_str());
                    message = "failed";
                }
            } 
            else{
                ROS_INFO("(New Map) SAME IDS ");
                message = "done 0";
            }

             /// Clear the used variables
            gotMapData = 0;
            mapId = "";
            mapDate = "";
            mapMetadata = "";
            map.clear();

            /// Send a message to the application to tell we finished
            boost::asio::write(*sock, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), error);

            if(error) 
                ROS_INFO("(New Map) Error : %s", error.message().c_str());
            else if(message ==  "done 1"){
                ROS_INFO("(New Map) Message sent succesfully : %lu bytes sent", message.length());
                //disconnect all other users to receive new map
                ROS_INFO("(New Map) Disconnect other uses to update them new map.");
                //disconnect all server
                ros::service::call("/gobot_software/disconnet_servers",empty_srv);
            }   
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
        //~ROS_INFO("(New Map) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();
        if(!sockets.count(ip))
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
        else
            ROS_ERROR("(New Map) the ip %s is already connected, this should not happen", ip.c_str());
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
        //~ROS_WARN("(New Map) The ip %s just disconnected", msg->data.c_str());
    }
    socketsMutex.unlock();
}

/*********************************** SHUT DOWN ***********************************/
void mySigintHandler(int sig){ 

    ros::shutdown();
}

/*********************************** MAIN ***********************************/

int main(int argc, char **argv){

    ros::init(argc, argv, "read_new_map");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);

    n.getParam("simulation", simulation);
    ROS_INFO("(New Map) simulation : %d", simulation);
    //Startup begin
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(120.0));
    //Startup end

    /// Subscribe to know when we disconnected from the server
    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);

    /// Advertise that we are going to publish to /map
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);

    ROS_INFO("(Read New Map) Ready to be launched.");

    std::thread t(server);

    ros::spin();

    return 0;
}
