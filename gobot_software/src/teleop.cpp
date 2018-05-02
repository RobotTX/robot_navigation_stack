#include "gobot_software/teleop.hpp"

#define TELEOP_PORT 5602

using boost::asio::ip::tcp;

const int max_length = 1024;

ros::Publisher teleop_pub;
ros::Publisher stop_pub;

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

void teleop(const int8_t val){
    ROS_INFO("(TELE_CONTROL) got data %d", val);
    double speed = 0.2;
    double turnSpeed = 0.5;
    // x == 1 -> forward       x == -1 -> backward
    // th == 1 -> left         th == -1 -> right
    int x(0), th(0);
    /// the value we got determine which way we go
    switch(val){
        case 0: /// Forward + Left
            x = 1;
            th = 1;
        break;
        case 1: /// Forward
            x = 1;
            th = 0;
        break;
        case 2: /// Forward + right
            x = 1;
            th = -1;
        break;
        case 3: /// Left
            x = 0;
            th = 1;
        break;
        case 5: /// Right
            x = 0;
            th = -1;
        break;
        case 6: /// Backward + left
            x = -1;
            th = -1;
        break;
        case 7: /// Backward
            x = -1;
            th = 0;
        break;
        case 8: /// Backward + right
            x = -1;
            th = 1;
        break;
        default: /// Stop
            x = 0;
            th = 0;
        break;
    }

    /// Before sending the teleoperation command, we stop all potential goals
    actionlib_msgs::GoalID cancel;
    cancel.stamp = ros::Time::now();
    cancel.id = "map";

    stop_pub.publish(cancel);
    
    /// Send the teleoperation command
    geometry_msgs::Twist twist;
    twist.linear.x = x * speed;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turnSpeed;

    teleop_pub.publish(twist);
}

void session(boost::shared_ptr<tcp::socket> sock){
    std::string ip = sock->remote_endpoint().address().to_string();
    //~ROS_INFO("(TELE_CONTROL) session launched %s", ip.c_str());

    while(ros::ok() && sockets.count(ip)){
        char data[max_length];

        boost::system::error_code error;
        size_t length = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            //~ROS_INFO("(TELE_CONTROL) Connection closed");
            return;
        } 
        else if (error){
            throw boost::system::system_error(error); // Some other error.
            return;
        }

        teleop(static_cast<int8_t>(data[0]));
    }
}

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), TELEOP_PORT));
    while(ros::ok()) {

        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));

        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        //~ROS_INFO("(TELE_CONTROL) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();

        if(sockets.find(ip)==sockets.end()){ //not find the ip
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
        }
        else{   //ip exists in the list
            sockets.find(ip)->second->close();
            sockets.find(ip)->second = sock;
        }
        socketsMutex.unlock();

        /// Launch the session thread which wait for a new map
        std::thread(session, sock).detach();
    }
}

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(msg->data)){
        sockets.at(msg->data)->close();
        sockets.erase(msg->data);
        //~ROS_WARN("(TELE_CONTROL) The ip %s just disconnected", msg->data.c_str());
    }
    socketsMutex.unlock();
}

/*********************************** MAIN ***********************************/

int main(int argc, char **argv){

    ros::init(argc, argv, "teleop");
    ros::NodeHandle n;
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    //Startup end

    ROS_INFO("(TELE_CONTROL) Ready to be launched.");

    /// Subscribe to know when we disconnect from the server
    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1, serverDisconnected);

    /// Advertise that we are going to publish to /cmd_vel & /move_base/cancel
    teleop_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    stop_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    std::thread t(server);

    ros::spin();

    return 0;
}
