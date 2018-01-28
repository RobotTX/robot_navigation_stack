#include "gobot_software/ping_server_new.hpp"

#define PING_THRESHOLD 5

static const std::string sep = std::string(1, 31);
std::string pingFile, ipsFile, wifiFile;

std::string battery_level = "51.0";

bool simulation = false, muteFlag = false, scanIP = false;
double STATUS_UPDATE=5.0, IP_UPDATE=20.0;

std::vector<std::string> availableIPs, connectedIPs;
std::vector<std::pair<std::string, int>> oldIPs;
std::mutex serverMutex, connectedMutex, ipMutex;
ros::Publisher disco_pub;
gobot_msg_srv::GetGobotStatus get_gobot_status;
std_srvs::Empty empty_srv;

/**
 * Create the string with information to connect to the Qt app
 * hostname + sep + stage + sep + batteryLevel + sep + muteFlag + sep + dockStatus
 */
std::string getDataToSend(void){
    /// Retrieves the hostname
    gobot_msg_srv::GetString get_name;
    ros::service::call("/gobot_status/get_name",get_name);

    /// Retrieve the docking status
    gobot_msg_srv::GetInt get_dock_status;
    ros::service::call("/gobot_status/get_dock_status", get_dock_status);
    
    /// Retrieves the path stage
    gobot_msg_srv::GetInt get_stage;
    ros::service::call("/gobot_status/get_stage", get_stage);
    
    //Retrives mute or not
    gobot_msg_srv::GetInt get_mute;
    ros::service::call("/gobot_status/get_mute",get_mute);
    muteFlag=(get_mute.response.data) ? 1 : 0;

    //Retrives battery percentage
    gobot_msg_srv::GetInt get_battery;
    ros::service::call("/gobot_status/battery_percent", get_battery);

    /// Form the string to send to the Qt app
    return  get_name.response.data + sep + std::to_string(get_stage.response.data) + sep + std::to_string(get_battery.response.data) + 
            sep + std::to_string(muteFlag) + sep + std::to_string(get_dock_status.response.data);
}

/**
 * Will disconnet all servers for updating map/wifi
 */
bool disServersSrvCallback(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    std_msgs::String msg;
    bool keep_ip;
    //disconnect all sockets when closed
    for(int i=0;i<oldIPs.size();i++){
        msg.data = oldIPs.at(i).first;
        keep_ip = false;
        for(int i=0; i<req.data.size();i++){
            if(msg.data.compare(req.data[i])==0){
                keep_ip = true;
                break;
            }
        }
        if(!keep_ip)
            disco_pub.publish(msg);
    }
    
    return true;
}

/**
 * Will update gobot status when receive any command
 */
bool updataStatusSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::thread([](){
        if(oldIPs.size()>0){
            std::string dataToSend = getDataToSend();
            std::vector<std::thread> data_threads;
            ipMutex.lock();
            //ROS_INFO("(ping_server) Trying to ping everyone %lu", availableIPs.size());
            for(int i = 0; i < oldIPs.size(); ++i)
                data_threads.push_back(std::thread(pingIP2, oldIPs.at(i).first, dataToSend, 2.5));
            ipMutex.unlock();

            ///we wait for all the threads to be done
            for(int i = 0; i < data_threads.size(); ++i)
                data_threads.at(i).join();          
        }
    }).detach();

    return true;
}

/**
 * Will ping all available IP addresses and send a message when we disconnect to one
 */
void pingAllIPs(void){
    /// Ping all servers every 5 secs
    ros::Rate loop_rate(1/STATUS_UPDATE);

    std::vector<std::thread> threads;

    while(ros::ok()){ 
        threads.clear();
        connectedIPs.clear();
        
        if(availableIPs.size()>0){
            /// Get the data to send to the Qt app
            std::string dataToSend = getDataToSend();
            //ROS_INFO("(ping_server) Data to send : %s", dataToSend.c_str());
            /// We create threads to ping every IP adress
            /// => threads reduce the required time to ping from ~12 sec to 2 sec
            serverMutex.lock();
            //ROS_INFO("(ping_server) Trying to ping everyone %lu", availableIPs.size());
            for(int i = 0; i < availableIPs.size(); ++i)
                threads.push_back(std::thread(pingIP, availableIPs.at(i), dataToSend, 2.5));
            serverMutex.unlock();

            /// We join all the threads => we wait for all the threads to be done
            for(int i = 0; i < threads.size(); ++i)
                threads.at(i).join();

            //ROS_INFO("(ping_server) Done pinging everyone");

            updateIP();
        }

        loop_rate.sleep();        
    }
}

//update old IP list with the now-connected IPs
void updateIP(){
    /// We check the oldIPs vs the new connected IP so we now which one disconnected
    std::vector<int> toRemove;
    ipMutex.lock();
    for(int i = 0; i < oldIPs.size(); ++i){
        bool still_connected = false;
        for(int j = 0; j < connectedIPs.size(); ++j)
            if(oldIPs.at(i).first.compare(connectedIPs.at(j)) == 0)
                still_connected = true;
        if(still_connected)
            oldIPs.at(i).second = 0;
        else {
            oldIPs.at(i).second++;
            if(oldIPs.at(i).second >= PING_THRESHOLD){
                /// Publish a message to the topic /gobot_software/server_disconnected with the IP address of the server which disconnected
                ROS_WARN("(ping_server) The ip %s disconnected", oldIPs.at(i).first.c_str());
                std_msgs::String msg;
                msg.data = oldIPs.at(i).first;
                disco_pub.publish(msg);
                toRemove.push_back(i);
            } 
            else{
                //ROS_WARN("Could not ping IP %s for %d time(s)", oldIPs.at(i).first.c_str(), oldIPs.at(i).second);
            }
        }
    }

    /// We remove all the disconnected IPs from the oldIPs vector
    for(int i = toRemove.size()-1; i >= 0; i--)
        oldIPs.erase(oldIPs.begin() + toRemove.at(i));

    /// We had the newly connectedIPs to the oldIPs vector
    for(int i = 0; i < connectedIPs.size(); ++i){
        bool hasIP = false;
        for(int j = 0; j < oldIPs.size(); ++j)
            if(connectedIPs.at(i).compare(oldIPs.at(j).first) == 0)
                hasIP = true;
        if(!hasIP)
            oldIPs.push_back(std::pair<std::string, int>(connectedIPs.at(i), 0));
    }
    ipMutex.unlock();
}

/**
 * Try to connect to the given IP, if can, then read OK, then send some data
 */
void pingIP(std::string ip, std::string dataToSend, double sec){
    //ROS_INFO("(ping_server) Called pingIP : %s", ip.c_str());
    timeout::tcp::Client client;
    try {
        /// Try to connect to the remote Qt app
        //tx??//close client connection after timeout
        client.connect(ip, "6000", boost::posix_time::seconds(sec));
        /// If we succesfully connect to the server, then the server is supposed to send us "OK"
        std::string read = client.read_some();
        if(read.compare("OK") == 0){
            /// Send the required data
            client.write_line(dataToSend, boost::posix_time::seconds(sec));
            /// Save the IP we just connected to in an array for later
            connectedMutex.lock();
            connectedIPs.push_back(ip);
            connectedMutex.unlock();
            //ROS_INFO("(ping_server) Connected to %s", ip.c_str());
        } //else
            //ROS_ERROR("(ping_server) read %lu bytes : %s", read.size(), read.c_str());
        
    } catch(std::exception& e) {
        //ROS_ERROR("(ping_server) error %s : %s", ip.c_str(), e.what());
    }
}

void pingIP2(std::string ip, std::string dataToSend, double sec){
    //ROS_INFO("(ping_server) Called pingIP : %s", ip.c_str());
    timeout::tcp::Client client;
    try {
        /// Try to connect to the remote Qt app
        //tx??//close client connection after timeout
        client.connect(ip, "6000", boost::posix_time::seconds(sec));
        /// If we succesfully connect to the server, then the server is supposed to send us "OK"
        std::string read = client.read_some();
        if(read.compare("OK") == 0){
            /// Send the required data
            client.write_line(dataToSend, boost::posix_time::seconds(sec));
            //ROS_INFO("(ping_server) Connected to %s", ip.c_str());
        } 

    } catch(std::exception& e) {
        //ROS_ERROR("(ping_server) error %s : %s", ip.c_str(), e.what());
    }
}


/**
 * Check all the IP addresses we can find on the local network and put them in an array
 * Usually take 5 to 10 secs
 */
void checkNewServers(void){
    /// We check for new IP addresses every 30 secs
    ros::Rate loop_rate(1/IP_UPDATE);

    while(ros::ok()){ 
        ros::spinOnce();
        scanIP=true;
        //ROS_INFO("(ping_server) Refreshing the list of potential servers");
        /// Script which will check all the IP on the local network and put them in a file
        const std::string ping_script = "sudo sh " + pingFile + " " + ipsFile +" " +wifiFile;
        system(ping_script.c_str());

        /// Get the file with the available IP addresses
        std::ifstream ifs(ipsFile, std::ifstream::in);
        if(ifs){
            std::string currentIP;
            serverMutex.lock();
            /// Save all the IP addresses in an array
            availableIPs.clear();
            while(std::getline(ifs, currentIP) && ros::ok())
                availableIPs.push_back(currentIP);
            serverMutex.unlock();
        }
        scanIP=false;
        //ROS_INFO("(ping_server) Available IPs: %lu, Connected IPs: %lu", availableIPs.size(),oldIPs.size());
        loop_rate.sleep();        
    }
}


void mySigintHandler(int sig)
{   
    ipMutex.lock();
    //disconnect all sockets when closed
    for(int i=0;i<oldIPs.size();i++){
        std_msgs::String msg;
        msg.data = oldIPs.at(i).first;
        disco_pub.publish(msg);
    }
    ipMutex.unlock();

    while(scanIP){
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ros::shutdown();
}

bool initParams(){
    ros::NodeHandle nh;
    if(nh.hasParam("simulation")){
        nh.getParam("simulation", simulation);
        ROS_INFO("(ping_server) simulation : %d", simulation);
    }
    nh.getParam("ips_file", ipsFile);
    nh.getParam("ping_file", pingFile);
    nh.getParam("wifi_file", wifiFile);
    nh.getParam("status_update", STATUS_UPDATE);
    nh.getParam("ip_update", IP_UPDATE);

    return true;
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "ping_server");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    
    if (initParams()){
        if(!simulation){
            //Startup begin
            ROS_INFO("(ping_server) Waiting for Robot finding initial pose...");
            ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(60.0));
            ROS_INFO("(ping_server) Robot finding initial pose is ready.");
        }

        //Startup end

        disco_pub = n.advertise<std_msgs::String>("/gobot_software/server_disconnected", 10);
        ros::ServiceServer updataStatusSrv = n.advertiseService("/gobot_software/update_status", updataStatusSrvCallback);
        ros::ServiceServer disServersSrv = n.advertiseService("/gobot_software/disconnet_servers", disServersSrvCallback);

        /// Thread which will get an array of potential servers
        std::thread t1(checkNewServers);
        ROS_INFO("(ping_server) checkNewServers thread launched");

        /// We try to ping all the available IPs
        std::thread t2(pingAllIPs);
        ROS_INFO("(ping_server) pingAllIPs thread launched");
        ros::spin();
    }

    return 0;
}