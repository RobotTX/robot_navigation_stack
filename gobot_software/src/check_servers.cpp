#include <iostream>
#include <string>
#include <signal.h>
#include <ros/ros.h>
#include <fstream>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <gobot_msg_srv/StringArrayMsg.h>

std::string pingFile, ipsFile, wifiFile;

double IP_UPDATE=20.0;

void initParams(){
    ros::NodeHandle nh;
    nh.getParam("ips_file", ipsFile);
    nh.getParam("ping_file", pingFile);
    nh.getParam("wifi_file", wifiFile);
    nh.getParam("ip_update", IP_UPDATE);
}

bool networkReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return true;
}

void mySigintHandler(int sig)
{   
    ros::shutdown();
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "check_servers");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    
    std::cout<<"###############START NETWORK INITILIZATION###############"<<std::endl;
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(CHECK_SERVERS) Waiting for Robot finding initial pose...");
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    ROS_INFO("(CHECK_SERVERS) Robot finding initial pose is ready.");

    //start network-manager when startup
    std::string cmd = "sudo service network-manager start";
    system(cmd.c_str());

    ros::ServiceServer networkReadySrv = n.advertiseService("/gobot_startup/network_ready", networkReadySrvCallback);
    //Startup end
    std::cout<<"###############COMPLETE NETWORK INITILIZATION###############"<<std::endl;

    initParams();

    ros::Publisher servers_pub = n.advertise<gobot_msg_srv::StringArrayMsg>("/gobot_software/online_servers", 1000);
    /**
    * Check all the IP addresses we can find on the local network and put them in an array
    * Usually take 5 to 10 secs
    */
    ros::Rate loop_rate(1/IP_UPDATE);
    while(ros::ok()){ 
        //ROS_INFO("(CHECK_SERVERS) Refreshing the list of potential servers");
        /// Script which will check all the IP on the local network and put them in a file
        const std::string ping_script = "sudo sh " + pingFile + " " + ipsFile +" " +wifiFile;
        system(ping_script.c_str());
        gobot_msg_srv::StringArrayMsg servers_ip_msg;
        /// Get the file with the available IP addresses
        std::ifstream ifs(ipsFile, std::ifstream::in);
        if(ifs){
            std::string currentIP;
            /// Save all the IP addresses in an array
            while(std::getline(ifs, currentIP) && ros::ok())
                servers_ip_msg.data.push_back(currentIP);
            ifs.close();
        }
        servers_pub.publish(servers_ip_msg);
        //ROS_INFO("(CHECK_SERVERS) Available IPs: %lu, Connected IPs: %lu", availableIPs.size(),oldIPs.size());
        ros::spinOnce();
        loop_rate.sleep();        
    }

    return 0;
}