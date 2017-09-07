#include "robot_pos_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_robot(io_service);
tcp::acceptor m_acceptor(io_service);

ros::Subscriber sub_robot;

#define max_length 1024

// reads on the socket looking for an ACK from the app that the recovered position has been received
void readAck(const std::string position_to_send){
	/// TODO probably needs to disappear or to be adapted as a new way to recover the position is needed
	bool found_ack(false);
	while(!found_ack){
		try {
			boost::system::error_code error;
			char data[max_length] = {0};
			size_t length = socket_robot.read_some(boost::asio::buffer(data), error);
			// parse what has been received
			std::istringstream iss(data);
			while(iss){
				std::string ack;
				iss >> ack;
                if(ack.compare(""))
                    ROS_INFO("(Robot Pos) potential ack %s", ack.c_str());
				if(ack.compare("ack") == 0)
					found_ack = true;
			}
			if(!found_ack){
				boost::asio::write(socket_robot, boost::asio::buffer(position_to_send), boost::asio::transfer_all(), error);
				ROS_INFO("(Robot Pos) confirmPositionRecovered resent status to application");
			}

		} catch (std::exception& e) {
			ROS_INFO("(Robot Pos) Exception : %s", e.what());
		}
	}
}

// called by recover position to notify the application that the position has been recovered
bool confirmPositionRecovered(gobot_software::RecoveredPosition::Request &req, gobot_software::RecoveredPosition::Response &res){
	tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(req.x, req.y, req.z, req.w));
 	tfScalar roll;
	tfScalar pitch;
	tfScalar yaw;
	matrix.getRPY(roll, pitch, yaw);
	// spaces at the beginning and at the end are useful to separate two messages 
	std::string position = " found " + std::to_string(req.x) + " " + std::to_string(req.y) + " " + std::to_string(yaw) + " ";
	try {
		boost::system::error_code ignored_error;
		boost::asio::write(socket_robot, boost::asio::buffer(position), boost::asio::transfer_all(), ignored_error);
		ROS_INFO("(Robot Pos) confirmPositionRecovered sent status to application");
	} catch (std::exception& e) {
		ROS_ERROR("(Robot Pos) Exception : %s", e.what());
	}

	// reads until ACK is found
	boost::thread t(boost::bind(readAck, position));

	return true;
}

void sendRobotPos(const std::string& robot_string){
	try {
		boost::system::error_code ignored_error;
		boost::asio::write(socket_robot, boost::asio::buffer(robot_string), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		ROS_ERROR("(Robot Pos) Exception : %s", e.what());
	}
}

void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg){
	/// to recover the orientation of the robot :)
	tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
 	tfScalar roll;
	tfScalar pitch;
	tfScalar yaw;
	matrix.getRPY(roll, pitch, yaw);
	std::string robot_string = std::to_string(msg->position.x) + " " + std::to_string(msg->position.y) + " " + std::to_string(yaw) + " ";
	sendRobotPos(robot_string);
	sleep(0.5);
}

bool startRobotPos(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	ROS_INFO("(Robot Pos) Starting robot_pos_sender");
	ros::NodeHandle n;

	int robotPort = req.port;

	if(socket_robot.is_open())
		socket_robot.close();

	if(m_acceptor.is_open())
		m_acceptor.close();

	socket_robot = tcp::socket(io_service);
	m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), robotPort));
	m_acceptor.set_option(tcp::acceptor::reuse_address(true));

	ROS_INFO("(Robot Pos) Connecting to ports : %d", robotPort);
	m_acceptor.accept(socket_robot);
	ROS_INFO("(Robot Pos) We are connected ");

	sub_robot = n.subscribe("/robot_pose", 1, getRobotPos);

	return true;
}

bool stopRobotPos(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	
	ROS_INFO("(Robot Pos) Stopping robot_pos_sender");

	sub_robot.shutdown();
	socket_robot.close();
	m_acceptor.close();

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "robot_pos_transfer");
	ROS_INFO("(Robot Pos) Ready to be launched.");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);
	
	ros::ServiceServer start_service = n.advertiseService("start_robot_pos_sender", startRobotPos);
	ros::ServiceServer stop_service = n.advertiseService("stop_robot_pos_sender", stopRobotPos);

	// to tell the application that the position of the robot has been recovered
	ros::ServiceServer service = n.advertiseService("send_position_recovered_confirmation", confirmPositionRecovered);

	// we wait for 10 sec
	ros::service::waitForService("send_position_recovered_confirmation", 10);
	
	if(ros::service::exists("send_position_recovered_confirmation", true))
		ROS_INFO("send_position_recovered_confirmation is available");
	else
		ROS_INFO("Oops send_position_recovered_confirmation is not available");

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
