#include "map_metadata_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_meta(io_service);
ros::Subscriber sub_meta;
tcp::acceptor m_acceptor(io_service);

void sendMetaData(const std::string& metadata_string){
	//std::cout << "(Map Metadata) Trying to send map_metadata : " << metadata_string << std::endl;
	try {
		boost::system::error_code ignored_error;
		int sizeSent = boost::asio::write(socket_meta, boost::asio::buffer(metadata_string, metadata_string.length()), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		std::cerr << "(Map Metadata) " << e.what() << std::endl;
	}
}

void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg){
	std::string metadata_string = std::to_string(msg->width) + " " + std::to_string(msg->height) + " " + std::to_string(msg->resolution) + " " + 
	std::to_string(msg->origin.position.x) + " " + std::to_string(msg->origin.position.y) + " ";
	sendMetaData(metadata_string);
}

bool startMeta(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Map Metadata) Starting map_metadata_sender" << std::endl;
	ros::NodeHandle n;

	int metaPort = req.port;	

	if(socket_meta.is_open())
		socket_meta.close();
	if(m_acceptor.is_open())
		m_acceptor.close();

	socket_meta = tcp::socket(io_service);
	m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), metaPort));
	m_acceptor.set_option(tcp::acceptor::reuse_address(true));

	std::cout << "(Map Metadata) Connecting to ports : " << metaPort << std::endl;
	m_acceptor.accept(socket_meta);
	std::cout << "(Map Metadata) We are connected " << std::endl;

	sub_meta = n.subscribe("/map_metadata", 1, getMetaData);

	return true;
}

bool stopMeta(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res){
	std::cout << "(Map Metadata) Stopping map_metadata_sender" << std::endl;

	sub_meta.shutdown();
	socket_meta.close();
	m_acceptor.close();

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "map_metadata_transfer");
	std::cout << "(Map Metadata) Ready to be launched." << std::endl;

	ros::NodeHandle n;

	ros::ServiceServer start_service = n.advertiseService("start_map_metadata_sender", startMeta);
	ros::ServiceServer stop_service = n.advertiseService("stop_map_metadata_sender", stopMeta);

	ros::spin();

	return 0;
}
