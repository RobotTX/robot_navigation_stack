#include "gobot_software/map_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_map(io_service);
ros::Subscriber sub_map;
tcp::acceptor m_acceptor(io_service);

#define HIGH_THRESHOLD 0.65*100
#define LOW_THRESHOLD 0.196*100

// this allows us to resub to the /map topic in case the connection would have been lost
bool sendingMapWhileScanning = false;
double map_resolution = 0.02;
double map_origin_x = -1;
double map_origin_y = -1;

void sendMap(const std::vector<uint8_t>& my_map){
	try {
		boost::system::error_code ignored_error;
		ROS_INFO("(Map::sendMap) Map size to send in uint8_t : %lu", my_map.size());
		boost::asio::write(socket_map, boost::asio::buffer(my_map), boost::asio::transfer_all(), ignored_error);
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
	int map_size = msg->info.width * msg->info.height;
	ROS_INFO("(Map::getMap) Just received a new map [%d, %d] => %d", msg->info.width, msg->info.height, map_size);
	sendMap(compress(msg->data, msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y, 0));
}

/// TODO can we remove that ? and all that is related to particle cloud
void getLocalMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int map_size = msg->info.width * msg->info.height;
	ROS_INFO("(Map::getLocalMap) Just received a new map [%d, %d] => %d", msg->info.width, msg->info.height, map_size);
	sendMap(compress(msg->data, msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y, 3));
}

// algorithm to compress the map before sending it
std::vector<uint8_t> compress(const std::vector<int8_t> map, const int map_width, const int map_height, const double map_resolution, const double _map_orixin_x, const double _map_orixin_y, const int who){
	std::vector<uint8_t> my_map;
	int last(205);
	uint32_t count(0);

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
	   	std::ifstream ifMap(mapIdFile, std::ifstream::in);
	   	
	   	std::string mapDate("0");
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
		
	} else if(who == 3) {
		/// when we are recovering the position, we need to send the size of the local map too
		/// we send it in a byte array so we cut our int into 4 bytes
		my_map.push_back((map_width & 0xff000000) >> 24);
		my_map.push_back((map_width & 0x00ff0000) >> 16);
		my_map.push_back((map_width & 0x0000ff00) >> 8);
		my_map.push_back((map_width & 0x000000ff));

		my_map.push_back((map_height & 0xff000000) >> 24);
		my_map.push_back((map_height & 0x00ff0000) >> 16);
		my_map.push_back((map_height & 0x0000ff00) >> 8);
		my_map.push_back((map_height & 0x000000ff));
	}


	int map_size = map_width * map_height;
    ROS_INFO("(Map::compress) Map size %d vs %lu", map_size, map.size());
	for(size_t i = 0; i < map_size; i++){
        int curr = 0;
        if(i >= map.size())
            curr = 205;
        else
            curr = map.at(i);

		if(who == 0 || who == 3){
		    if(curr < 0)
	            curr = 205;
	        else if(curr < LOW_THRESHOLD)
	            curr = 255;
	        else if(curr < HIGH_THRESHOLD)
	            curr = 205;
	        else 
	            curr = 0;
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
	else if(who == 3)
		my_map.push_back(251);
	else
		my_map.push_back(253);

	return my_map;
}

bool startMap(gobot_msg_srv::Port::Request &req,
    gobot_msg_srv::Port::Response &res){
	ROS_INFO("(Map::startMap) Starting map_sender");

	int mapPort = req.port;	

	if(socket_map.is_open())
		socket_map.close();

	if(m_acceptor.is_open())
		m_acceptor.close();

	socket_map = tcp::socket(io_service);
	m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), mapPort));
	m_acceptor.set_option(tcp::acceptor::reuse_address(true));

	ROS_INFO("(Map::startMap) Connecting to ports : %d", mapPort);
	m_acceptor.accept(socket_map);
	ROS_INFO("(Map::startMap) We are connected");

	// if the robot disconnects while scanning it unsuscribes to /map
	// startMap is called when the robot reconnects and we need to resub
	if(sendingMapWhileScanning){
		ros::NodeHandle n;
		sub_map = n.subscribe("/map", 1, getMap);
	}

	return true;
}

bool sendAutoMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("(Map::sendAutoMap) SendAutoMap");
	ros::NodeHandle n;
	sub_map.shutdown();
	sub_map = n.subscribe("/map", 1, getMap);
	sendingMapWhileScanning = true;
	return true;
}

bool sendLocalMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	ROS_INFO("(Map::sendLocalMap) sendLocalMap");

	ros::NodeHandle n;
	/// in case sub map would have subscribed to another topic before we unsubscribe first
	sub_map.shutdown();
	sub_map = n.subscribe("/move_base/local_costmap/costmap", 1, getLocalMap);
	return true;
}

bool stopAutoMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("(Map::stopAutoMap) StopAutoMap");

	sub_map.shutdown();
	sendingMapWhileScanning = false;

	return true;
}

bool stopSendingLocalMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response &res){
	ROS_INFO("(Map::stopSendingLocalMap) StopSendingLocalMap");

	sub_map.shutdown();

	return true;
}

// who:
// 0 : scan 
// 1 : application requesting at connection time
// 2 : to merge
// 3 : recovering position
bool sendOnceMap(gobot_msg_srv::Port::Request &req,
    gobot_msg_srv::Port::Response &res){
	ROS_INFO("(Map::sendOnceMap) SendOnceMap doing nothing for now");

	int who = req.port;	

	std::vector<int8_t> my_map;
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
				my_map.push_back(static_cast<int8_t>(line.at(i)));

		mapFile.close();
		ROS_INFO("(Map::sendOnceMap) Got the whole map from file, about to compress and send it %lu", my_map.size());
		sendMap(compress(my_map, width, height, map_resolution, map_origin_x, map_origin_y, who));

		return true;
		
	} else
		return false;
}

bool stopMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	ROS_INFO("(Map::stopMap) Stopping map_sender");
	sub_map.shutdown();
	socket_map.close();
	m_acceptor.close();
	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "map_transfer");
	ROS_INFO("(Map::main) Ready to be launched.");

	ros::NodeHandle n;

	ros::ServiceServer start_service = n.advertiseService("start_map_sender", startMap);
	ros::ServiceServer send_once_service = n.advertiseService("send_once_map_sender", sendOnceMap);
	ros::ServiceServer send_auto_service = n.advertiseService("send_auto_map_sender", sendAutoMap);
	ros::ServiceServer stop_auto_service = n.advertiseService("stop_auto_map_sender", stopAutoMap);
	ros::ServiceServer stop_service = n.advertiseService("stop_map_sender", stopMap);

	// to recover a robot's position
	ros::ServiceServer send_local_map_service = n.advertiseService("send_local_map", sendLocalMap);
	ros::ServiceServer stop_sending_local_map_service = n.advertiseService("stop_sending_local_map", stopSendingLocalMap);
	ros::Subscriber sub_meta = n.subscribe("/map_metadata", 1, getMetaData);

	ros::Rate loop_rate(20);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
