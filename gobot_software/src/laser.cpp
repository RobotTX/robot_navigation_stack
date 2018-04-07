#include "gobot_software/laser.hpp"

#define LASER_PORT 5605

using boost::asio::ip::tcp;

const int max_length = 1024;

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

ros::Subscriber sub_laser;

void getLaserData(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(sockets.size() > 0){
        std::vector<float> scan;
        scan.push_back(msg->angle_min);
        scan.push_back(msg->angle_max);
        scan.push_back(msg->angle_increment);
        for(int i = 0; i < (msg->angle_max - msg->angle_min) / msg->angle_increment; i++)
            scan.push_back(msg->ranges[i]);
        scan.push_back(-1.0f);

        sendLaserData(scan);
    }
}

void sendLaserData(const std::vector<float>& scan){
    try {
        socketsMutex.lock();
        /// We send the position of the robot to every Qt app
        for(auto const &elem : sockets)
            boost::asio::write(*(elem.second), boost::asio::buffer(scan, scan.size()));
        socketsMutex.unlock();
    } catch (std::exception& e) {
        e.what();
    }
}

bool sendLaserService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("(LASER_TRANSFER) send_laser_data_sender");
    ros::NodeHandle n;
    sub_laser.shutdown();
    sub_laser = n.subscribe("/scan", 1, getLaserData);
    return true;
}

bool stopSendLaserService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("(LASER_TRANSFER) stop_send_laser_data_sender");
    sub_laser.shutdown();
    return true;
}

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), LASER_PORT));
    while(ros::ok()) {

        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));

        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        ROS_INFO("(LASER_TRANSFER) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();
        if(!sockets.count(ip))
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
        else
            ROS_ERROR("(LASER_TRANSFER) the ip %s is already connected, this should not happen", ip.c_str());
        socketsMutex.unlock();
    }
}

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(msg->data)){
        sockets.at(msg->data)->close();
        sockets.erase(msg->data);
        ROS_WARN("(LASER_TRANSFER) The ip %s just disconnected", msg->data.c_str());
    }
    socketsMutex.unlock();
}

/*********************************** MAIN ***********************************/

void mySigintHandler(int sig){  
    sub_laser.shutdown();
    ros::shutdown();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "laser_data_transfer");
    ROS_INFO("(LASER_TRANSFER) Ready to be launched.");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);

    ros::ServiceServer send_service = n.advertiseService("send_laser_data_sender", sendLaserService);
    ros::ServiceServer stop_send_service = n.advertiseService("stop_send_laser_data_sender", stopSendLaserService);

    std::thread t(server);

    ros::spin();

    return 0;
}