#include "map_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_map(io_service);
ros::Subscriber sub_map;
tcp::acceptor m_acceptor(io_service);

#define HIGH_THRESHOLD 0.65*100
#define LOW_THRESHOLD 0.196*100

// this allows us to resub to the /map topic in case the connection would have been lost
bool sendingMapWhileScanning = false;
std::string metadata_string = "";

void sendMap(const std::vector<uint8_t>& my_map){
	try {
		boost::system::error_code ignored_error;
		std::cout << "(Map) Map size to send in uint8_t : " << my_map.size() << std::endl;
		boost::asio::write(socket_map, boost::asio::buffer(my_map), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		e.what();
	}
}

void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg){
	metadata_string = std::to_string(msg->width) + " " + std::to_string(msg->height) + " " + std::to_string(msg->resolution) + " " + 
	                  std::to_string(msg->origin.position.x) + " " + std::to_string(msg->origin.position.y);
}

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int map_size = msg->info.width * msg->info.height;
	std::cout << "(Map) Just received a new map" << msg->info.width << " " << msg->info.height << " " << map_size << std::endl;
	sendMap(compress(msg->data, msg->info.width, msg->info.height, 0));
}

/// TODO can we remove that ? and all that is related to particle cloud
void getLocalMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int map_size = msg->info.width * msg->info.height;
	std::cout << "(Map) Just received a new map" << msg->info.width << " " << msg->info.height << " " << map_size << std::endl;
	sendMap(compress(msg->data, msg->info.width, msg->info.height, 3));
}

// algorithm to compress the map before sending it
std::vector<uint8_t> compress(const std::vector<int8_t> map, const int map_width, const int map_height, const int who){
	std::vector<uint8_t> my_map;
	int last(205);
	uint32_t count(0);

	if(who == 0){
		/// If we are scanning a new map, we send the published metadata with it
	   	std::cout << "(Map) Map metadata (who = 0) : " << metadata_string << std::endl;
	   	for(int i = 0; i < metadata_string.size(); i++)
	   		my_map.push_back(static_cast<uint8_t>(metadata_string.at(i)));
	   	
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
			std::cout << "map_transfer got map id file " << mapIdFile << std::endl;
		}
		std::string mapId("{0}");
	   	std::ifstream ifMap(mapIdFile, std::ifstream::in);
	   	
	   	std::string mapDate("0");
	   	if(ifMap){
	   		getline(ifMap, mapId);
	   		getline(ifMap, mapDate);
	   		ifMap.close();
	   	}

	   	std::string str = mapId + " " + mapDate + " " + metadata_string;
	   	std::cout << "(Map) Map metadata (who = 1 or 2) : " << str << std::endl;
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
	for(size_t i = 0; i < map_size; i++){
		int curr = map.at(i);

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

bool startMap(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Map) Starting map_sender" << std::endl;

	int mapPort = req.port;	

	if(socket_map.is_open())
		socket_map.close();

	if(m_acceptor.is_open())
		m_acceptor.close();

	socket_map = tcp::socket(io_service);
	m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), mapPort));
	m_acceptor.set_option(tcp::acceptor::reuse_address(true));

	std::cout << "(Map) Connecting to ports : " << mapPort << std::endl;
	m_acceptor.accept(socket_map);
	std::cout << "(Map) We are connected " << std::endl;

	// if the robot disconnects while scanning it unsuscribes to /map
	// startMap is called when the robot reconnects and we need to resub
	if(sendingMapWhileScanning){
		ros::NodeHandle n;
		sub_map = n.subscribe("/map", 1, getMap);
	}

	return true;
}

bool sendAutoMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(Map) SendAutoMap " << std::endl;
	ros::NodeHandle n;
	sub_map.shutdown();
	sub_map = n.subscribe("/map", 1, getMap);
	sendingMapWhileScanning = true;
	return true;
}

bool sendLocalMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	std::cout << "(Map) sendLocalMap " << std::endl;

	ros::NodeHandle n;
	/// in case sub map would have subscribed to another topic before we unsubscribe first
	sub_map.shutdown();
	sub_map = n.subscribe("/move_base/local_costmap/costmap", 1, getLocalMap);
	return true;
}

bool stopAutoMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(Map) StopAutoMap " << std::endl;

	sub_map.shutdown();
	sendingMapWhileScanning = false;

	return true;
}

bool stopSendingLocalMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response &res){
	std::cout << "(Map) StopSendingLocalMap " << std::endl;

	sub_map.shutdown();

	return true;
}

// who:
// 0 : scan 
// 1 : application requesting at connection time
// 2 : to merge
// 3 : recovering position
bool sendOnceMap(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Map) SendOnceMap doing nothing for now" << std::endl;

	int who = req.port;	

	std::vector<int8_t> my_map;
    std::string mapFileStr;
    ros::NodeHandle n;
    if(n.hasParam("map_image_used")){
    	n.getParam("map_image_used", mapFileStr);
    	std::cout << "map_transfer set map image file to " << mapFileStr << std::endl;
    }

	std::string line;
	std::ifstream mapFile;

	mapFile.open(mapFileStr);
	if(mapFile.is_open()){
		getline(mapFile, line);
		std::cout << "(Map) 1 : " << line << std::endl;

		getline(mapFile, line);
        if(line.at(0) == '#'){
            std::cout << "(Map) Got a second line with # : " << line << std::endl;
            getline(mapFile, line);
        }

		std::cout << "(Map) 2 : " << line << std::endl;

		int width = std::stoi(line.substr(0,line.find_first_of(" ")));
		int height = std::stoi(line);
		int map_size = width * height;
		std::cout << "(Map) width : " << width << "\n(Map) height : " << height << "\n(Map) size : " << map_size << std::endl;

		getline(mapFile, line);
		std::cout << "(Map) 3 : " << line << std::endl;

		while(getline(mapFile, line)){
			for(int i = 0; i < line.size(); i++){
				my_map.push_back(static_cast<int8_t>(line.at(i)));
			}
		}

		mapFile.close();
		std::cout << "(Map) Got the whole map from file, about to compress and send it" << std::endl;
		sendMap(compress(my_map, width, height, who));

		return true;
		
	} else
		return false;
}

bool stopMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(Map) Stopping map_sender" << std::endl;
	sub_map.shutdown();
	socket_map.close();
	m_acceptor.close();
	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "map_transfer");
	std::cout << "(Map) Ready to be launched." << std::endl;

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
