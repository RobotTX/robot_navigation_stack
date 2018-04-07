#include "gobot_software/ping_servers.hpp"

#define PING_THRESHOLD 5

static const std::string sep = std::string(1, 31);

bool muteFlag = false;
double STATUS_UPDATE=5.0, WAITING_TIME=5.0;

std::vector<std::string> availableIPs, connectedIPs;
std::vector<std::pair<std::string, int>> oldIPs;
std::mutex serverMutex, connectedMutex, ipMutex, sendDataMutex;
ros::Publisher disco_pub, host_pub;
std_srvs::Empty empty_srv;
ros::Time disco_time;

robot_class::GetRobot GetRobot;

/**
 * Create the string with information to connect to the Qt app
 * hostname + sep + stage + sep + batteryLevel + sep + muteFlag + sep + dockStatus
**/
std::string getDataToSend(void){
    gobot_msg_srv::GetString get_update_status;
    ros::service::call("/gobot_status/get_update_status", get_update_status);

    /// Form the string to send to the Qt app
    return  get_update_status.response.data;
}

/**
 * Will disconnet all servers for updating map/wifi
**/
bool disServersSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ipMutex.lock();
    //disconnect all sockets when closed
    for(int i=0;i<oldIPs.size();i++){
        std_msgs::String msg;
        msg.data = oldIPs.at(i).first;
        disco_pub.publish(msg);
    }
    oldIPs.clear();
    ipMutex.unlock();

    //clear all IPs
    serverMutex.lock();
    availableIPs.clear();
    serverMutex.unlock();

    disco_time = ros::Time::now();

    return true;
}

/**
 * Will update gobot status when receive any command
**/
void updateInfoCallback(const std_msgs::String::ConstPtr& update_status){
    std::vector<std::thread> data_threads;
    ipMutex.lock();
    if(oldIPs.size()>0){
        if((ros::Time::now()-disco_time).toSec()>WAITING_TIME){
            std::string dataToSend = update_status->data;
            for(int i = 0; i < oldIPs.size(); ++i)
                data_threads.push_back(std::thread(pingIP2, oldIPs.at(i).first, dataToSend, 3.0));
        }
    }
    ipMutex.unlock();
    
    sendDataMutex.lock();
    if(data_threads.size()>0){
        ///we wait for all the threads to be done
        for(int i = 0; i < data_threads.size(); ++i)
            data_threads.at(i).join();  
    }
    sendDataMutex.unlock();
}

/**
 * Will update servers' IP address in local network
**/
void updateServersCallback(const gobot_msg_srv::StringArrayMsg::ConstPtr& servers_msg){
    serverMutex.lock();
    availableIPs.clear();
    for(int i=0;i<servers_msg->data.size();i++){
        availableIPs.push_back(servers_msg->data[i]);
    }
    serverMutex.unlock();
}


/**
 * update old IP list with the now-connected IPs
**/
void updateIP(){
    /// We check the oldIPs vs the new connected IP so we now which one disconnected
    std::vector<int> toRemove;
    ipMutex.lock();
    for(int i = 0; i < oldIPs.size(); ++i){
        bool still_connected = false;
        for(int j = 0; j < connectedIPs.size(); ++j){
            if(oldIPs.at(i).first.compare(connectedIPs.at(j)) == 0){
                still_connected = true;
                break;
            }
        }
        if(still_connected)
            oldIPs.at(i).second = 0;
        else {
            oldIPs.at(i).second++;
            if(oldIPs.at(i).second >= PING_THRESHOLD){
                /// Publish a message to the topic /gobot_software/server_disconnected with the IP address of the server which disconnected
                ROS_WARN("(PING_SERVERS) The ip %s disconnected", oldIPs.at(i).first.c_str());
                std_msgs::String msg;
                msg.data = oldIPs.at(i).first;
                disco_pub.publish(msg);
                toRemove.push_back(i);
            } 
            else{
                //ROS_WARN("(PING_SERVERS) Could not ping IP %s for %d time(s)", oldIPs.at(i).first.c_str(), oldIPs.at(i).second);
            }
        }
    }

    /// We remove all the disconnected IPs from the oldIPs vector
    for(int i = toRemove.size()-1; i >= 0; i--)
        oldIPs.erase(oldIPs.begin() + toRemove.at(i));

    /// We had the newly connectedIPs to the oldIPs vector
    for(int i = 0; i < connectedIPs.size(); ++i){
        bool hasIP = false;
        for(int j = 0; j < oldIPs.size(); ++j){
            if(connectedIPs.at(i).compare(oldIPs.at(j).first) == 0){
                hasIP = true;
                break;
            }
        }
        if(!hasIP)
            oldIPs.push_back(std::pair<std::string, int>(connectedIPs.at(i), 0));
    }

    gobot_msg_srv::StringArrayMsg host_ip_msg;
    for(int i = 0; i < oldIPs.size(); ++i){
        host_ip_msg.data.push_back(oldIPs.at(i).first);
    }
    host_pub.publish(host_ip_msg);

    ipMutex.unlock();
}

/**
 * Try to connect to the given IP, if can, then read OK, then send some data
 */
void pingIP(std::string ip, std::string dataToSend, double sec){
    //ROS_INFO("(PING_SERVERS) Called pingIP : %s", ip.c_str());
    timeout::tcp::Client client;
    try {
        /// Try to connect to the remote Qt app
        client.connect(ip, "6000", boost::posix_time::seconds(sec));
        /// If we succesfully connect to the server, then the server is supposed to send us "OK"
        std::string read = client.read_some();
        if(read.compare("OK") == 0){
            /// Send the required data
            client.write_line(dataToSend, boost::posix_time::seconds(2.5));
            /// Save the IP we just connected to in an array for later
            connectedMutex.lock();
            connectedIPs.push_back(ip);
            connectedMutex.unlock();
            //ROS_INFO("(PING_SERVERS) Connected to %s", ip.c_str());
        } //else
            //ROS_ERROR("(PING_SERVERS) read %lu bytes : %s", read.size(), read.c_str());
        
    } catch(std::exception& e) {
        //ROS_ERROR("(PING_SERVERS) error %s : %s", ip.c_str(), e.what());
    }
}

void pingIP2(std::string ip, std::string dataToSend, double sec){
    //ROS_INFO("(PING_SERVERS) Called pingIP : %s", ip.c_str());
    timeout::tcp::Client client;
    try {
        /// Try to connect to the remote Qt app
        client.connect(ip, "6000", boost::posix_time::seconds(sec));
        /// If we succesfully connect to the server, then the server is supposed to send us "OK"
        std::string read = client.read_some();
        if(read.compare("OK") == 0){
            /// Send the required data
            client.write_line(dataToSend, boost::posix_time::seconds(2.5));
            //ROS_INFO("(PING_SERVERS) Connected to %s", ip.c_str());
        } 

    } catch(std::exception& e) {
        //ROS_ERROR("(PING_SERVERS) error %s : %s", ip.c_str(), e.what());
    }
}


void initParams(){
    ros::NodeHandle nh;
    nh.getParam("status_update", STATUS_UPDATE);
    nh.getParam("waiting_time", WAITING_TIME);
    disco_time = ros::Time::now();
}


void mySigintHandler(int sig)
{   
    disco_time = ros::Time::now();
    ipMutex.lock();
    //disconnect all sockets when closed
    for(int i=0;i<oldIPs.size();i++){
        std_msgs::String msg;
        msg.data = oldIPs.at(i).first;
        disco_pub.publish(msg);
    }
    oldIPs.clear();
    ipMutex.unlock();

    ros::shutdown();
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "ping_server");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    
    //Startup begin
    ROS_INFO("(PING_SERVERS) Waiting for Robot network ready...");
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    ROS_INFO("(PING_SERVERS) Robot network is ready.");
    //Startup end

    initParams();

    ros::ServiceServer disServersSrv = n.advertiseService("/gobot_software/disconnet_servers", disServersSrvCallback);

    disco_pub = n.advertise<std_msgs::String>("/gobot_software/server_disconnected", 1);

    host_pub = n.advertise<gobot_msg_srv::StringArrayMsg>("/gobot_software/connected_servers", 1);

    ros::Subscriber update_sub = n.subscribe("/gobot_status/update_information", 1, updateInfoCallback);

    ros::Subscriber servers_sub = n.subscribe("/gobot_software/online_servers", 1, updateServersCallback);


    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();

    /**
    * Will ping all available IP addresses and send a message when we disconnect to one
    */
    /// Ping all servers every 5 secs
    ros::Rate loop_rate(1/STATUS_UPDATE);

    std::vector<std::thread> threads;

    while(ros::ok()){ 
        threads.clear();
        connectedIPs.clear();
        //ROS_INFO("(PING_SERVERS) %f",(ros::Time::now()-disco_time).toSec());
        serverMutex.lock();
        if(availableIPs.size()>0){
            //ROS_INFO("(PING_SERVERS) Data to send : %s", dataToSend.c_str());
            /// We create threads to ping every IP adress
            /// => threads reduce the required time to ping from ~12 sec to 2 sec
            if((ros::Time::now()-disco_time).toSec()>WAITING_TIME){
                /// Get the data to send to the Qt app
                std::string dataToSend = getDataToSend();
                //ROS_INFO("(PING_SERVERS) Trying to ping everyone %lu", availableIPs.size());
                for(int i = 0; i < availableIPs.size(); ++i)
                    threads.push_back(std::thread(pingIP, availableIPs.at(i), dataToSend, 3.0));
            }
        }
        serverMutex.unlock();

        sendDataMutex.lock();
        if(threads.size()>0){
            /// We join all the threads => we wait for all the threads to be done
            for(int i = 0; i < threads.size(); ++i)
                threads.at(i).join();
            //ROS_INFO("(PING_SERVERS) Done pinging everyone");
            updateIP();
        }
        sendDataMutex.unlock();
        loop_rate.sleep();        
    }

    ros::waitForShutdown();
    return 0;
}