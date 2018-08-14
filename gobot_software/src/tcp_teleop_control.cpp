#include "gobot_software/tcp_teleop_control.hpp"

#define TELEOP_PORT 5602

using boost::asio::ip::tcp;

const int max_length = 1024;
double speed = 0.25, turnSpeed = 0.5;
bool teleop_on = false;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;
std::mutex socketsMutex;
ros::Publisher teleop_pub, stop_pub;
ros::Time last_signal_time;

void send_speed(int linear, int angular){
    //if command is stop robot, set teleop to be false
    if(linear == 0 && angular == 0){
        teleop_on = false;
    }
    else{
        teleop_on = true;
    }
    //update the last signal time
    last_signal_time = ros::Time::now();

    /// Send the teleoperation command
    geometry_msgs::Twist twist;
    twist.linear.x = linear * speed;
    twist.angular.z = angular * turnSpeed;

    teleop_pub.publish(twist);
}

void tcp_teleop_control(const int8_t val){
    ROS_INFO("(TELE_CONTROL) Received data %d", val);
    // x == 1 -> forward       x == -1 -> backward
    // th == 1 -> left             th == -1 -> right
    int x = 0, th = 0;
    /// the value we got determine which way we go
    switch(val){
        case 1: /// Forward
            x = 1;
            th = 0;
        break;
        case 3: /// Left
            x = 0;
            th = 1;
        break;
        case 5: /// Right
            x = 0;
            th = -1;
        break;
        case 7: /// Backward
            x = -1;
            th = 0;
        break;
        default: /// Stop
            x = 0;
            th = 0;
        break;
    }

    send_speed(x, th);
}

void session(boost::shared_ptr<tcp::socket> sock){
    std::string ip = sock->remote_endpoint().address().to_string();
    //~ROS_INFO("(TELE_CONTROL) session launched %s", ip.c_str());
    /// Before sending the teleoperation command, we stop all potential goals
    actionlib_msgs::GoalID cancel;
    cancel.stamp = ros::Time::now();
    cancel.id = "map";
    stop_pub.publish(cancel);
    
    while(ros::ok() && sockets.count(ip)){
        char data[max_length];

        boost::system::error_code error;
        size_t length = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            //~ROS_INFO("(TELE_CONTROL) Connection closed");
            if(teleop_on)
                send_speed(0, 0);
            return;
        } 
        else if (error){
            if(teleop_on)
                send_speed(0, 0);
            throw boost::system::system_error(error); // Some other error.
            return;
        }

        tcp_teleop_control(static_cast<int8_t>(data[0]));
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
//timer to check whether the signal is received continuously
void timerCallback(const ros::TimerEvent&){
    //if have servers connected and teleop is on
    if(sockets.size()>0 && teleop_on){
        //if not received motion message for more than 1.5 seconds, stop the robot
        if(ros::Time::now()-last_signal_time > ros::Duration(1.5)){
            send_speed(0, 0);
        }
    }
}

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

    ros::init(argc, argv, "tcp_teleop_control");
    ros::NodeHandle n;
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    //Startup end

    ROS_INFO("(TELE_CONTROL) Ready to be launched.");

    /// Subscribe to know when we disconnect from the server
    ros::Subscriber sub = n.subscribe("/gobot_software/server_disconnected", 1, serverDisconnected);

    /// Advertise that we are going to publish to /teleop_cmd_vel & /move_base/cancel
    teleop_pub = n.advertise<geometry_msgs::Twist>("/teleop_cmd_vel", 1);
    stop_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    ros::Timer signal_timer = n.createTimer(ros::Duration(1.0), timerCallback);

    std::thread t(server);

    ros::spin();

    return 0;
}
