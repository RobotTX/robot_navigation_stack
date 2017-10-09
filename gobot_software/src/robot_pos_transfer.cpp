#include "gobot_software/robot_pos_transfer.hpp"

#define PORT_ROBOT_POS 4001

using boost::asio::ip::tcp;

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;
std::string robot_string;

ros::Publisher disco_pub;

void sendRobotPos(const ros::TimerEvent&){
    if(sockets.size() > 0){
        socketsMutex.lock();
        /// We send the position of the robot to every Qt app
        for(auto const &elem : sockets){
            try {
                boost::asio::write(*(elem.second), boost::asio::buffer(robot_string, robot_string.length()));
            } catch (std::exception& e) {
                ROS_ERROR("(Robot Pos) Exception %s : %s", elem.first.c_str(), e.what());
                std_msgs::String msg;
                msg.data = elem.first;
                disco_pub.publish(msg);
            }
        }
        socketsMutex.unlock();
    }
}

void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg){
    /// to recover the orientation of the robot :)
    tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
    tfScalar roll;
    tfScalar pitch;
    tfScalar yaw;
    matrix.getRPY(roll, pitch, yaw);
    robot_string = std::to_string(msg->position.x) + " " + std::to_string(msg->position.y) + " " + std::to_string(yaw) + " ";
}

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), PORT_ROBOT_POS));
    while(ros::ok()) {

        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));
        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        ROS_INFO("(Robot Pos) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();
        if(!sockets.count(ip))
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
        else
            ROS_ERROR("(Robot Pos) the ip %s is already connected, this should not happen", ip.c_str());
        socketsMutex.unlock();
    }
}

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    ROS_WARN("(Robot Pos) The ip %s just disconnected", msg->data.c_str());
    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(msg->data)){
        sockets.at(msg->data)->close();
        sockets.erase(msg->data);
    }
    socketsMutex.unlock();
}


/*********************************** MAIN ***********************************/

int main(int argc, char **argv){

	ros::init(argc, argv, "robot_pos_transfer");
	ROS_INFO("(Robot Pos) Ready to be launched.");

	ros::NodeHandle n;

    disco_pub = n.advertise<std_msgs::String>("server_disconnected", 10);
	
    //Periodically send robot pose to connected clients
    ros::Timer timer = n.createTimer(ros::Duration(1), sendRobotPos);

    ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);

    ros::Subscriber sub_robot = n.subscribe("/robot_pose", 1, getRobotPos);

    std::thread t(server);

    ros::spin();

	return 0;
}

