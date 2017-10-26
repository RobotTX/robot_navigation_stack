#include "gobot_software/read_new_map.hpp"

#define NEW_MAP_PORT 5601

using boost::asio::ip::tcp;

const int max_length = 1024;
bool simulation = false;

std::mutex socketsMutex;
std::mutex mapMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

ros::Publisher map_pub;


void session(boost::shared_ptr<tcp::socket> sock){

    std::string ip = sock->remote_endpoint().address().to_string();
    ROS_INFO("(New Map) session launched %s", ip.c_str());

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
            ROS_INFO("(New Map) Connection closed");
            return;
        } else if (error) {
            throw boost::system::system_error(error); // Some other error.
            return;
        }

       /// Parse the data as we are supposed to receive : "mapId ; mapDate ; metadata ; map"
        for(int i = 0; i < length; i++){
            if(data[i] == ';' && gotMapData <= 2){
                /// The first ; means we got the mapId
                /// the second means we got the metadata
                /// the third means we got the map date
                ROS_INFO("(New Map) ';' found");
                gotMapData++;
            } else {
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
            ROS_INFO("(New Map) Last separator found");


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

            if(mapIdFromFile.compare(mapId) == 0)
                ROS_INFO("(New Map) SAME IDS ");

            mapMutex.lock();
            /// If we have a different map, we replace it
            if(mapIdFromFile.compare(mapId) != 0){
                /// Save the id of the new map
                ROS_INFO("(New Map) Id of the new map : %s", mapId.c_str());
                ROS_INFO("(New Map) Date of the new map : %s", mapDate.c_str());

                std::string mapIdFile;
                if(n.hasParam("map_id_file")){
                    n.getParam("map_id_file", mapIdFile);
                    ROS_INFO("read_new_map set mapIdFile to %s", mapIdFile.c_str());
                }

                std::ofstream ofs(mapIdFile, std::ofstream::out | std::ofstream::trunc);
                if(ofs){
                    ofs << mapId << std::endl << mapDate << std::endl;
                    ofs.close();
                    ROS_INFO("(New Map) Map id updated : %s with date %s", mapId.c_str(), mapDate.c_str());

                    /// Set the medatada of the new map
                    ROS_INFO("(New Map) Map metadata before split : %s", mapMetadata.c_str());
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

                    /*
                    // if initPosX <= 100.0 it means we have just finished a scan and we have the position of the robot
                    if(initPosX > -100.0){

                        if(n.hasParam("robot_position_file")){
                            std::string robotPositionFile;
                            n.getParam("robot_position_file", robotPositionFile);
                            ofs.open(robotPositionFile, std::ifstream::out | std::ofstream::trunc);
                            ofs << 0;
                            ofs.close();
                        }

                        std::string initialPoseFile;
                        if(n.hasParam("last_known_position_file")){
                            n.getParam("last_known_position_file", initialPoseFile);
                            ROS_INFO("read_new_map set last known position file to %s", initialPoseFile.c_str());
                        } 

                        ofs.open(initialPoseFile, std::ofstream::out | std::ofstream::trunc);
                        if(ofs.is_open()){
                            /// We translate the rotation of the robot from degrees to a quaternion
                            tf::Quaternion quaternion;
                            quaternion.setEuler(0, 0, -orientation*3.14159/180);

                            /// We write the inital position of the robot in its file
                            ofs << initPosX << " " << initPosY << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w() << std::endl;

                            ofs.close();

                        } else {
                            ROS_INFO("(New Map) Could not open the file to create a new initialPoseFile file %s", initialPoseFile.c_str());
                            message = "failed";
                        }
                    }
                    */ 


                    /// Update the config file
                    std::string mapConfig;
                    if(n.hasParam("map_config_used")){
                        n.getParam("map_config_used", mapConfig);
                        ROS_INFO("read new map set map config to %s", mapConfig.c_str());
                    } 
                    ofs.open(mapConfig, std::ofstream::out | std::ofstream::trunc);
                    if(ofs.is_open()){
                        std::string resolutionStr = "resolution: " + std::to_string(resolution);
                        std::string originStr = "origin: [" + std::to_string(initPosX) + ", " + std::to_string(initPosY) + ", 0.00]";

                        ofs << "image: used_map.pgm" << std::endl << resolutionStr << std::endl << originStr << std::endl << "negate: 0" << std::endl << "occupied_thresh: 0.65" << std::endl << "free_thresh: 0.196";
                        ofs.close(); 
                    }

                    /// We save the file in a the pgm file used by amcl
                    std::string mapFile;
                    if(n.hasParam("map_image_used")){
                        n.getParam("map_image_used", mapFile);
                        ROS_INFO("read new map set map file to %s", mapFile.c_str());
                    } 
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

                        /// Kill gobot move so that we'll restart it with the new map
                        std::string cmd = "rosnode kill /move_base";
                        system(cmd.c_str());

                        ROS_INFO("(New Map) We killed gobot_navigation");

                        /// We delete the old path
                        std::string pathFile;
                        if(n.hasParam("path_file")){
                            n.getParam("path_file", pathFile);
                            ROS_INFO("read new map set path file to %s", pathFile.c_str());
                        }
                        ofs.open(pathFile, std::ofstream::out | std::ofstream::trunc);
                        ofs.close();
                        ROS_INFO("(New Map) Path deleted");

                        /// We delete the old path stage
                        std::string pathStageFile;
                        if(n.hasParam("path_stage_file")){
                            n.getParam("path_stage_file", pathStageFile);
                            ROS_INFO("read new map set path stage file to %s", pathStageFile.c_str());
                        }
                        ofs.open(pathStageFile, std::ofstream::out | std::ofstream::trunc);
                        ofs.close();
                        ROS_INFO("(New Map) Path stage deleted");

                        /// We delete the old home
                        std::string homeFile;
                        if(n.hasParam("home_file")){
                            n.getParam("home_file", homeFile);
                            ROS_INFO("read new map home file to %s", homeFile.c_str());
                        }
                        ofs.open(homeFile, std::ofstream::out | std::ofstream::trunc);
                        ofs.close();
                        ROS_INFO("(New Map) Home deleted");

                        /// We detele the loop
                        std::string loopFile;
                        if(n.hasParam("path_loop_file")){
                            n.getParam("path_loop_file", loopFile);
                            ROS_INFO("read new map loop file to %s", loopFile.c_str());
                        }
                        ofs.open(loopFile, std::ofstream::out | std::ofstream::trunc);
                        ofs << 0;
                        ofs.close();
                        ROS_INFO("(New Map) Loop deleted");

                        
                        /// Relaunch gobot_navigation
                        if(simulation)
                            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gazebo_slam.launch\"";
                        else
                            cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source /home/gtdollar/catkin_ws/devel/setup.bash;roslaunch gobot_navigation gobot_navigation.launch\"";
                        sleep(5);
                        system(cmd.c_str());

                        ROS_INFO("(New Map) We relaunched gobot_navigation");
                        message = "done 1";

                    } else {
                        ROS_INFO("(New Map) Could not open the file to create a new pgm file %s", mapFile.c_str());
                        message = "failed";
                    }
                } else {
                    ROS_INFO("(New Map) Map id could not be updated : %s with date %s", mapId.c_str(), mapDate.c_str());
                    message = "failed";
                }
            } else {
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
            else 
                ROS_INFO("(New Map) Message sent succesfully : %lu bytes sent", message.length());


            mapMutex.unlock();
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
        ROS_INFO("(New Map) Command socket connected to %s", ip.c_str());
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

/*********************************** MAIN ***********************************/

int main(int argc, char **argv){

    ros::init(argc, argv, "read_new_map");
    ros::NodeHandle n;
    
    ROS_INFO("(New Map) Ready to be launched.");

    /// Subscribe to know when we disconnected from the server
    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);

    /// Advertise that we are going to publish to /map
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);

    n.param<bool>("simulation", simulation, false);
    ROS_INFO("(New Map) simulation : %d", simulation);

    std::thread t(server);

    ros::spin();

    return 0;
}
