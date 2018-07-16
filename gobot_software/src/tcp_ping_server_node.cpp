#include "gobot_software/tcp_ping_server.hpp"

int main(int argc, char* argv[]){

    ros::init(argc, argv, "tcp_ping_server_node");
    ros::NodeHandle n;

    std::cout<<"###############START PING SERVER INITILIZATION###############"<<std::endl;
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(PING_SERVERS) Waiting for Robot network ready...");
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    ROS_INFO("(PING_SERVERS) Robot network is ready.");
    //Startup end

    PingServerClass ping_server_start;
    std::cout<<"###############COMPLETE PING SERVER INITILIZATION###############"<<std::endl;
    
    ros::spin();
    return 0;
}