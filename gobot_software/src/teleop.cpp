#include "teleop.hpp"
#include <utility>

using boost::asio::ip::tcp;

const int max_length = 1024;
bool waiting = false;
bool connected = false;

boost::asio::io_service io_service;
tcp::socket socket_teleop(io_service);
tcp::acceptor m_acceptor(io_service);

ros::Publisher teleop_pub;
ros::Publisher stop_pub;

void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n){
    std::cout << "(Teleop) session launched" << std::endl;

    while(ros::ok() && connected){
        char data[max_length];

        boost::system::error_code error;
        size_t length = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            std::cout << "(Teleop) Connection closed" << std::endl;
            connected = false;
            return;
        } else if (error) {
            throw boost::system::system_error(error); // Some other error.
            return;
        }

        teleop(static_cast<int8_t>(data[0]));
    }
}

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n){
    std::cout << "(Teleop) Waiting for connection" << std::endl;

    boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(*io_service));

    m_acceptor->accept(*sock);
    std::cout << "(Teleop) Command socket connected to " << sock->remote_endpoint().address().to_string() << std::endl;
    connected = true;
    waiting = false;
    boost::thread t(boost::bind(session, sock, n));
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    if(connected){
        std::cout << "(Teleop) I heard " << std::endl;
        teleop(4);
        connected = false;
    }
}

void teleop(const int8_t val){
    std::cout << "(Teleop) got data " << static_cast<int> (val) << std::endl;
    if(connected){
        double speed = 0.2;
        double turnSpeed = 1.0;
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

        ros::spinOnce();
        
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
}

int main(int argc, char **argv){

    ros::init(argc, argv, "teleop");
    ros::NodeHandle n;
    
    std::cout << "(Teleop) Ready to be launched." << std::endl;

    /// Subscribe to know when we disconnect from the server
    ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);

    /// Advertise that we are going to publish to /cmd_vel & /move_base/cancel
    teleop_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    stop_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);

    boost::shared_ptr<boost::asio::io_service> io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
    io_service->run();

    boost::shared_ptr<tcp::endpoint> m_endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), PORT));
    boost::shared_ptr<tcp::acceptor> m_acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*io_service, *m_endpoint));

    m_acceptor->set_option(tcp::acceptor::reuse_address(true));

    ros::Rate r(10);

    while(ros::ok()){

        if(!connected && !waiting){
            
            std::cout << "(Teleop) Ready to connect" << std::endl;
            boost::thread t(boost::bind(asyncAccept, io_service, m_acceptor, n));

            waiting = true;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
