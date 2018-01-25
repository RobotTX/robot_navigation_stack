#include "gobot_software/map_transfer.hpp"

#define MAP_PORT 4002
#define HIGH_THRESHOLD 0.65*100
#define LOW_THRESHOLD 0.196*100

#define WHITE 255
#define GREY 205
#define BLACK 0

using boost::asio::ip::tcp;

double map_resolution = 0.02;
double map_origin_x = -1;
double map_origin_y = -1;


struct session_object {
    bool sendingMapWhileScanning = false;
    bool sendAutoMap = false;
};

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;
std::map<std::string, session_object> session_map;

ros::Subscriber sub_map;

void sendMap(const std::string ip, const std::vector<uint8_t>& my_map){
	try {
		boost::system::error_code ignored_error;
		ROS_INFO("(Map::sendMap) Map size to send in uint8_t : %lu", my_map.size());
        if(sockets.count(ip))
		  boost::asio::write(*(sockets.at(ip)), boost::asio::buffer(my_map), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		e.what();
	}
}

void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg){
	map_resolution = msg->resolution;
    map_origin_x = msg->origin.position.x;
    map_origin_y = msg->origin.position.y;
}

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    socketsMutex.lock();
    for(auto const &elem : sockets){
        if(session_map.count(elem.first) && session_map.at(elem.first).sendAutoMap){
            ROS_INFO("(Map::getMap) Just received a new map o send to the QT app [%d, %d] => %d", msg->info.width, msg->info.height, msg->info.width * msg->info.height);
            sendMap(elem.first, compress(msg->data, msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y, 0));
        }
    }
    socketsMutex.unlock();
}

// algorithm to compress the map before sending it
std::vector<uint8_t> compress(const std::vector<int8_t> map, const int map_width, const int map_height, const double map_resolution, const double _map_orixin_x, const double _map_orixin_y, const int who){
	std::vector<uint8_t> my_map;

	if(who == 0){
		/// If we are scanning a new map, we send the published metadata with it
	   	ROS_INFO("(Map::compress) Map metadata (who = 0) : [%d, %d] %f [%f, %f]", map_width, map_height, map_resolution, _map_orixin_x, _map_orixin_y);
        for(int i = 0; i < std::to_string(map_width).size(); ++i)
   		   my_map.push_back(static_cast<uint8_t>(std::to_string(map_width).at(i)));
        my_map.push_back(static_cast<uint8_t>(' '));

        for(int i = 0; i < std::to_string(map_height).size(); ++i)
            my_map.push_back(static_cast<uint8_t>(std::to_string(map_height).at(i)));
        my_map.push_back(static_cast<uint8_t>(' '));
        
        for(int i = 0; i < std::to_string(map_resolution).size(); ++i)
            my_map.push_back(static_cast<uint8_t>(std::to_string(map_resolution).at(i)));
        my_map.push_back(static_cast<uint8_t>(' '));
        
        for(int i = 0; i < std::to_string(_map_orixin_x).size(); ++i)
            my_map.push_back(static_cast<uint8_t>(std::to_string(_map_orixin_x).at(i)));
        my_map.push_back(static_cast<uint8_t>(' '));
        
        for(int i = 0; i < std::to_string(_map_orixin_y).size(); ++i)
            my_map.push_back(static_cast<uint8_t>(std::to_string(_map_orixin_y).at(i)));
	   	
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);

	} else if(who == 1 || who == 2){
		/// If the map comes from a pgm, we send the mapId, the mapDate,
		/// the resolution and the origin with it
		ros::NodeHandle n;
		std::string mapIdFile;
		if(n.hasParam("map_id_file")){
			n.getParam("map_id_file", mapIdFile);
			ROS_INFO("(Map::compress) map_transfer got map id file %s", mapIdFile.c_str());
		}
		std::string mapId("{0}");
		std::string mapDate("0");
	   	std::ifstream ifMap(mapIdFile, std::ifstream::in);
	
	   	if(ifMap){
	   		std::getline(ifMap, mapId);
	   		std::getline(ifMap, mapDate);
	   		ifMap.close();
	   	}

	   	std::string str = mapId + " " + mapDate + " " + std::to_string(map_width) + " " 
        + std::to_string(map_height) + " " + std::to_string(map_resolution) + " " 
        + std::to_string(_map_orixin_x) + " " + std::to_string(_map_orixin_y);

	   	ROS_INFO("(Map::compress) Map metadata (who = 1 or 2) : %s", str.c_str());
	   	for(int i = 0; i < str.size(); i++)
	   		my_map.push_back(static_cast<uint8_t>(str.at(i)));
	   	
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
		
	}


	int map_size = map_width * map_height;
	int last = GREY;
	uint32_t count=0;
	int curr = 0;
    ROS_INFO("(Map::compress) Map size %d vs %lu", map_size, map.size());
	for(size_t i = 0; i < map_size; i++){

        if(i >= map.size())
            curr = GREY;
        else
            curr = map.at(i);

		if(who == 0){
		    if(curr < 0)
	            curr = GREY;
	        else if(curr < LOW_THRESHOLD)
	            curr = WHITE;
	        else if(curr < HIGH_THRESHOLD)
	            curr = GREY;
	        else 
	            curr = BLACK;
	    }

		if(curr != last){
			my_map.push_back(static_cast<uint8_t>(last));
			my_map.push_back((count & 0xff000000) >> 24);
			my_map.push_back((count & 0x00ff0000) >> 16);
			my_map.push_back((count & 0x0000ff00) >> 8);
			my_map.push_back((count & 0x000000ff));

			last = curr;
			count = 0;
		}
		count++;
	}

	my_map.push_back(static_cast<uint8_t>(last));
	my_map.push_back((count & 0xff000000) >> 24);
	my_map.push_back((count & 0x00ff0000) >> 16);
	my_map.push_back((count & 0x0000ff00) >> 8);
	my_map.push_back((count & 0x000000ff));

	// the user knows that when 254 is encountered a map has entirely been received
	my_map.push_back(254);
	my_map.push_back(254);
	my_map.push_back(254);
	my_map.push_back(254);

	if(who == 1)
		my_map.push_back(254);
	else if(who == 2)
		my_map.push_back(252);
	else
		my_map.push_back(253);

	return my_map;
}

bool sendAutoMap(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
	ROS_INFO("(Map::sendAutoMap) SendAutoMap");
	ros::NodeHandle n;

    int count = 0;
    for(auto const &elem : session_map)
        count += elem.second.sendAutoMap;
    
    if(!count)
        sub_map = n.subscribe("/map", 1, getMap);

    session_map.at(req.data[0]).sendAutoMap = true;

	return true;
}


bool stopAutoMap(gobot_msg_srv::SetString::Request &req, gobot_msg_srv::SetString::Response &res){
	ROS_INFO("(Map::stopAutoMap) StopAutoMap");

    session_map.at(req.data[0]).sendAutoMap = false;

    int count = 0;
    for(auto const &elem : session_map)
        count += elem.second.sendAutoMap;
    
    if(!count)
        sub_map.shutdown();

	return true;
}

// who:
// 0 : scan 
// 1 : application requesting at connection time
// 2 : to merge
bool sendOnceMap(gobot_msg_srv::SendMap::Request &req,
    gobot_msg_srv::SendMap::Response &res){
	ROS_INFO("(Map::sendOnceMap) SendOnceMap doing nothing for now");

	int who = req.who;	

	std::vector<int8_t> ori_map;
    std::string mapFileStr;
    ros::NodeHandle n;
    if(n.hasParam("map_image_used")){
    	n.getParam("map_image_used", mapFileStr);
    	ROS_INFO("(Map::sendOnceMap) map_transfer set map image file to %s", mapFileStr.c_str());
    }

	std::string line;
	std::ifstream mapFile;

	mapFile.open(mapFileStr);
	if(mapFile.is_open()){
		std::getline(mapFile, line);
		ROS_INFO("(Map::sendOnceMap) 1 : %s", line.c_str());

		std::getline(mapFile, line);
        if(line.at(0) == '#'){
            ROS_INFO("(Map::sendOnceMap) Got a second line with # : %s", line.c_str());
            std::getline(mapFile, line);
        }

		ROS_INFO("(Map::sendOnceMap) 2 : %s", line.c_str());

		int width = std::stoi(line.substr(0,line.find_first_of(" ")));
		int height = std::stoi(line.substr(line.find_first_of(" "), line.length()));
		int map_size = width * height;
		ROS_INFO("(Map::sendOnceMap) width & height : [%d, %d] => %d", width, height, map_size);

		std::getline(mapFile, line);
		ROS_INFO("(Map::sendOnceMap) 3 : %s", line.c_str());

		while(std::getline(mapFile, line))
			for(int i = 0; i < line.size(); i++)
				ori_map.push_back(static_cast<int8_t>(line.at(i)));

		mapFile.close();
		
		ROS_INFO("(Map::sendOnceMap) Got the whole map from file, about to compress and send it %lu", ori_map.size());
		sendMap(req.ip, compress(ori_map, width, height, map_resolution, map_origin_x, map_origin_y, who));

		return true;
		
	} 
	else
		return false;
}

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), MAP_PORT));
    while(ros::ok()) {

		bool error = false;
        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));

        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        //~ROS_INFO("(Map::server) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();
        if(!sockets.count(ip)){
            session_object session;
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
            session_map.insert(std::pair<std::string, session_object>(ip, session));
        } 
		else{
			error = true;
            ROS_ERROR("(Map::server) the ip %s is already connected, this should not happen", ip.c_str());
		}
        socketsMutex.unlock();
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
    }
    socketsMutex.unlock();
}
/*********************************** SHUT DOWN ***********************************/
void mySigintHandler(int sig){ 

    ros::shutdown();
}

/*********************************** MAIN ***********************************/

int main(int argc, char **argv){

	ros::init(argc, argv, "map_transfer");
	ROS_INFO("(Map::main) Ready to be launched.");

	ros::NodeHandle n;
	signal(SIGINT, mySigintHandler);

	ros::ServiceServer send_once_service = n.advertiseService("/gobot_function/send_once_map_sender", sendOnceMap);
	ros::ServiceServer send_auto_service = n.advertiseService("/gobot_function/send_auto_map_sender", sendAutoMap);
	ros::ServiceServer stop_auto_service = n.advertiseService("/gobot_function/stop_auto_map_sender", stopAutoMap);

    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);

    ros::Subscriber sub_meta = n.subscribe("/map_metadata", 1, getMetaData);

    std::thread t(server);

	ros::spin();

	return 0;
}
