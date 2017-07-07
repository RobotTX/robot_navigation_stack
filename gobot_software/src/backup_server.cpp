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
	std::cout << "(Backup system) Waiting for connection" << std::endl;

	boost::shared_ptr<tcp::socket> socket = boost::shared_ptr<tcp::socket>(new tcp::socket(*io_service));

	m_acceptor->accept(*socket);
	std::cout << "(Backup system)  socket connected to " << socket->remote_endpoint().address().to_string() << std::endl;
	connected = true;
	waiting = false;

	// Notifies the application that we are connected
	sendMessageToApplication(socket, "Backup system connection established");

	boost::thread t(boost::bind(session, socket));
}

void sendMessageToApplication(boost::shared_ptr<tcp::socket> socket, const std::string message){
	
	std::cout << "(Backup system) Sending message : " << message << std::endl;

	try {
		boost::system::error_code ignored_error;
		boost::asio::write(*socket, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), ignored_error);
		std::cout << "(Backup system) Message sent succesfully" << std::endl;
	} catch (std::exception& e) {
		std::cerr << "(Backup system) Message not sent" << std::endl;
		std::cerr << e.what() << std::endl;
	}
}

void session(boost::shared_ptr<tcp::socket> socket){
	std::cout << "(Backup system) Waiting for a reboot order" << std::endl;
	try {

		std::string message("");

		while(ros::ok() && connected){
			char buffer[max_length] = {0};

			boost::system::error_code error;
			size_t length = socket->read_some(boost::asio::buffer(buffer), error);
			std::cout << "(Backup system) " << length << " byte(s) received" << std::endl;
			if (error == boost::asio::error::eof)
				std::cout << "(Backup system) Got error eof" << std::endl;
			
			if (error == boost::asio::error::connection_reset){
				std::cout << "(Backup system) Connection closed" << std::endl;
				disconnect();
        	} else if (error) 
				throw boost::system::system_error(error); // Some other error.

			for(int i = 0; i < length; i++){
				if(static_cast<int>(buffer[i]) != 0)
					message += buffer[i];
			}

			if(message.compare("reboot") == 0){
				std::cout << "calling reboot" << std::endl;
				std::string cmd = "rosnode kill /play_path";
	            system(cmd.c_str());
	            cmd = "rosnode kill /move_base";
	            system(cmd.c_str());
	            sleep(3);
	            system("sh ~/catkin_ws/src/gobot_software/src/start_gobot_move.sh &");
	            std::cout << "Relaunched gobot_move" << std::endl;
	            sleep(5);
	            system("sh ~/catkin_ws/src/gobot_software/src/start_gobot_software.sh &");
	            std::cout << "Relaunched gobot_software" << std::endl;
	            sleep(3);
			}
			message = "";
		}

	} catch (std::exception& e) {
		std::cerr << "(Backup system) Exception in thread: " << e.what() << "\n";
	}
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
	std::cout << "server disconnected callback called" << std::endl;
	disconnect();
}

void disconnect(void){
	if(connected){
		std::cout << "(Backup system) Robot could not find the application " << std::endl;
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
		std::cerr << "(Backup system) Exception: " << e.what() << std::endl;
	}

	return 0;
}