#include "read_new_map.hpp"

using boost::asio::ip::tcp;

const int max_length = 1024;
bool waiting = false;
bool connected = false;

boost::asio::io_service io_service;
tcp::socket socket_robot(io_service);
tcp::acceptor m_acceptor(io_service);

ros::Publisher map_pub;

void session(ros::NodeHandle n){
    
    std::cout << "(New Map) session launched" << std::endl;
    int gotMapData(0);
    std::string mapId("");
    std::string mapMetadata("");
    std::string mapDate("");
    std::vector<uint8_t> map;
    std::string message("done 0");

    while(ros::ok() && connected){
        
        // buffer in which we store the bytes we read on the socket
        char data[max_length];

        boost::system::error_code error;
        size_t length = socket_robot.read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            std::cout << "(New Map) Connection closed" << std::endl;
            connected = false;
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
                std::cout << "(New Map) ';' found" << std::endl;
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
            std::cout << "(New Map) Last separator found" << std::endl;


            /// We check if we already have a map with the same id
            std::string mapIdFile;
            if(n.hasParam("map_id_file")){
                n.getParam("map_id_file", mapIdFile);
                std::cout << "(New Map) got map id file " << mapIdFile << std::endl;
            }
            std::string mapIdFromFile("");
            std::ifstream ifMap(mapIdFile, std::ifstream::in);
            
            if(ifMap){
                getline(ifMap, mapIdFromFile);
                ifMap.close();
            }

            if(mapIdFromFile.compare(mapId) == 0)
                std::cout << " SAME IDS " << std::endl;

            /// If we have a different map, we replace it
            if(mapIdFromFile.compare(mapId) != 0){
                /// Save the id of the new map
                std::cout << "(New Map) Id of the new map : " << mapId << std::endl;
                std::cout << "(New Map) Date of the new map : " << mapDate << std::endl;
                std::string mapIdFile;
                if(n.hasParam("map_id_file")){
                    n.getParam("map_id_file", mapIdFile);
                    std::cout << "read_new_map set mapIdFile to " << mapIdFile << std::endl;
                }

                std::ofstream ofs(mapIdFile, std::ofstream::out | std::ofstream::trunc);
                if(ofs){
                    ofs << mapId << std::endl << mapDate << std::endl;
                    ofs.close();
                    std::cout << "(New Map) Map id updated : " << mapId << " with date " << mapDate << std::endl;

                    /// Set the medatada of the new map
                    std::cout << "(New Map) Map metadata before split : " << mapMetadata << std::endl;
                    int width(0);
                    int height(0);
                    double resolution(0.0f);
                    double initPosX(0.0f);
                    double initPosY(0.0f);
                    double orientation(0.0f);

                    std::istringstream iss(mapMetadata);
                    iss >> width >> height >> resolution >> initPosX >> initPosY >> orientation;
                    std::cout << "(New Map) Map metadata after split : " << width << " " << height << " " << resolution << " " << initPosX << " " << initPosY << " " << orientation << std::endl;

                    /// We remove the 5 last bytes as they are only there to identify the end of the map
                    map.erase(map.end() - 5, map.end());
                    std::cout << "(New Map) Size of the map received : " << map.size() << std::endl;

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
                            std::cout << "read_new_map set last known position file to " << initialPoseFile << std::endl;
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
                            std::cout << "(New Map) Could not open the file to create a new initialPoseFile file " << initialPoseFile << std::endl;
                            message = "failed";
                        }
                    } 

                    /// We save the file in a the pgm file used by amcl

                    std::string mapFile;
                    if(n.hasParam("map_image_used")){
                        n.getParam("map_image_used", mapFile);
                        std::cout << "read new map set map file to " << mapFile << std::endl;
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

                        std::cout << "(New Map) New map pgm file created in " << mapFile << std::endl;

                        /// Kill gobot move so that we'll restart it with the new map
                        std::string cmd = "rosnode kill /move_base";
                        system(cmd.c_str());

                        sleep(5);
                        std::cout << "(New Map) We killed gobot_move" << std::endl;

                        /// We delete the old path
                        std::string pathFile;
                        if(n.hasParam("path_file")){
                            n.getParam("path_file", pathFile);
                            std::cout << "read new map set path file to " << pathFile;
                        }
                        ofs.open(pathFile, std::ofstream::out | std::ofstream::trunc);
                        ofs.close();
                        std::cout << "(New Map) Path deleted" << std::endl;

                        /// We delete the old home
                        std::string homeFile;
                        if(n.hasParam("home_file")){
                            n.getParam("home_file", homeFile);
                            std::cout << "read new map home file to " << homeFile;
                        }
                        ofs.open(homeFile, std::ofstream::out | std::ofstream::trunc);
                        ofs.close();
                        std::cout << "(New Map) Home deleted" << std::endl;

                        /// Relaunch gobot_move
                        //cmd = "roslaunch gobot_move slam.launch &";
                        cmd = "roslaunch gobot_move gazebo_slam.launch &";
                        system(cmd.c_str());
                        std::cout << "(New Map) We relaunched gobot_move" << std::endl;
                        message = "done 1";

                    } else {
                        std::cout << "(New Map) Could not open the file to create a new pgm file " << mapFile << std::endl;
                        message = "failed";
                    }
                } else {
                    std::cout << "(New Map) Map id could not be updated : " << mapId << " with date " << mapDate << std::endl;
                    message = "failed";
                }
            } else {
                message = "done 0";
            }

            /// Clear the used variables
            gotMapData = 0;
            mapId = "";
            mapMetadata = "";
            map.clear();

            /// Send a message to the application to tell we finished
            boost::asio::write(socket_robot, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), error);

            if(error) 
                std::cout << "(New Map) Error : " << error.message() << std::endl;
            else 
                std::cout << "(New Map) Message sent succesfully : " << message.length() << " bytes sent" << std::endl;
        }
    }
}

void asyncAccept(ros::NodeHandle n){
    std::cout << "(New Map) Waiting for connection" << std::endl;

    if(socket_robot.is_open())
        socket_robot.close();

    if(m_acceptor.is_open())
        m_acceptor.close();

    socket_robot = tcp::socket(io_service);
    m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), PORT));
    m_acceptor.set_option(tcp::acceptor::reuse_address(true));

    m_acceptor.accept(socket_robot);
    std::cout << "(New Map) Command socket connected to " << socket_robot.remote_endpoint().address().to_string() << std::endl;
    connected = true;
    waiting = false;
    boost::thread t(boost::bind(session, n));
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    if(connected){
        std::cout << "(New Map) I heard " << std::endl;
        connected = false;
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "read_new_map");
    ros::NodeHandle n;
    
    std::cout << "(New Map) Ready to be launched." << std::endl;

    /// Subscribe to know when we disconnected from the server
    ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);

    /// Advertise that we are going to publish to /map
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);

    ros::Rate r(10);

    while(ros::ok()){
        if(!connected && !waiting){
            std::cout << "(New Map) Ready to connect" << std::endl;
            boost::thread t(boost::bind(asyncAccept, n));
            waiting = true;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
