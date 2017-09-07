#include "backup_server.hpp"

const int max_length = 512;

bool connected = false;
bool waiting = false;

void server(const unsigned short port){

	boost::shared_ptr<boost::asio::io_service> io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
	io_service->run();

	boost::shared_ptr<tcp::endpoint> m_endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), port));
	boost::shared_ptr<tcp::acceptor> m_acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*io_service, *m_endpoint));

	m_acceptor->set_option(tcp::acceptor::reuse_address(true));

	ros::Rate r(2);
	while(ros::ok()){
		if(!connected && !waiting){
			boost::thread t(boost::bind(asyncAccept, io_service, m_acceptor));
			waiting = true;
		}
		ros::spinOnce();
		r.sleep();
	}
}

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor){
	ROS_INFO("(Backup system) Waiting for connection");

	boost::shared_ptr<tcp::socket> socket = boost::shared_ptr<tcp::socket>(new tcp::socket(*io_service));

	m_acceptor->accept(*socket);
	ROS_INFO("(Backup system)  socket connected to %s", socket->remote_endpoint().address().to_string().c_str());
	connected = true;
	waiting = false;

	// Notifies the application that we are connected
	sendMessageToApplication(socket, "Backup system connection established");

	boost::thread t(boost::bind(session, socket));
}

void sendMessageToApplication(boost::shared_ptr<tcp::socket> socket, const std::string message){
	
	ROS_INFO("(Backup system) Sending message : %s", message.c_str());

	try {
		boost::system::error_code ignored_error;
		boost::asio::write(*socket, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), ignored_error);
		ROS_INFO("(Backup system) Message sent succesfully");
	} catch (std::exception& e) {
		ROS_ERROR("(Backup system) Message not sent : %s", e.what());
	}
}

void session(boost::shared_ptr<tcp::socket> socket){
	ROS_INFO("(Backup system) Waiting for a reboot order");
	try {

		std::string message("");

		while(ros::ok() && connected){
			char buffer[max_length] = {0};

			boost::system::error_code error;
			size_t length = socket->read_some(boost::asio::buffer(buffer), error);
			ROS_INFO("(Backup system) %d  byte(s) received", length);
			if (error == boost::asio::error::eof)
				ROS_INFO("(Backup system) Got error eof");
			
			if (error == boost::asio::error::connection_reset){
				ROS_INFO("(Backup system) Connection closed");
				disconnect();
        	} else if (error) 
				throw boost::system::system_error(error); // Some other error.

			for(int i = 0; i < length; i++){
				if(static_cast<int>(buffer[i]) != 0)
					message += buffer[i];
			}

			if(message.compare("reboot") == 0){
				ROS_INFO("calling reboot");
				std::string cmd = "rosnode kill /play_path";
	            system(cmd.c_str());
	            cmd = "rosnode kill /move_base";
	            system(cmd.c_str());
	            sleep(3);
	            system("sh ~/catkin_ws/src/gobot_software/src/start_gobot_move.sh &");
	            ROS_INFO("Relaunched gobot_move");
	            sleep(5);
	            system("sh ~/catkin_ws/src/gobot_software/src/start_gobot_software.sh &");
	            ROS_INFO("Relaunched gobot_software");
	            sleep(3);
			}
			message = "";
		}

	} catch (std::exception& e) {
		ROS_ERROR("(Backup system) Exception in thread : %s", e.what());
	}
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("server disconnected callback called");
	disconnect();
}

void disconnect(void){
	if(connected){
		ROS_INFO("(Backup system) Robot could not find the application ");
		connected = false;
	}
}

int main(int argc, char* argv[]){

	try {

		ros::init(argc, argv, "backup_system");
		ros::NodeHandle n;
  		ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);

		ros::spinOnce();

		server(CONNECTION_PORT);
		
	} catch (std::exception& e) {
		ROS_ERROR("(Backup system) Exception : %s", e.what());
	}

	return 0;
}