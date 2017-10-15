#include "gobot_software/ping_server_new.hpp"

#define PING_THRESHOLD 3

static const std::string sep = std::string(1, 31);
std::string pingFile;
std::string ipsFile;

bool simulation = false;
bool chargingFlag = false;
int batteryLevel = 50;

std::vector<std::string> availableIPs;
std::vector<std::string> connectedIPs;
std::vector<std::pair<std::string, int>> oldIPs;
std::mutex serverMutex;
std::mutex connectedMutex;

ros::Publisher disco_pub;
ros::ServiceClient getGobotStatusSrv;
gobot_msg_srv::GetGobotStatus get_gobot_status;

/**
 * Create the string with information to connect to the Qt app
 * hostname + sep + stage + sep + batteryLevel + sep + chargingFlag + sep + dockStatus
 */
std::string getDataToSend(void){
    /// Retrieves the hostname
    gobot_msg_srv::GetString get_name;
    ros::service::call("/gobot_status/get_name",get_name);

    /// Retrieve the docking status
    gobot_msg_srv::GetDockStatus get_dock_status;
    ros::service::call("/gobot_status/get_dock_status", get_dock_status);
    
    /// Retrieves the path stage
    gobot_msg_srv::GetStage get_stage;
    ros::service::call("/gobot_status/get_stage", get_stage);
    chargingFlag=get_dock_status.response.status==3 ? true : false;

    /// Form the string to send to the Qt app
    return get_name.response.data[0] + sep + std::to_string(get_stage.response.stage) + sep + std::to_string(batteryLevel) + sep + std::to_string(chargingFlag) + sep + std::to_string(get_dock_status.response.status);
}

/**
 * Will ping all available IP addresses and send a message when we disconnect to one
 */
void pingAllIPs(void){
    /// Ping all servers every 3 secs
    //tx??//detect disconnection after 6 seconds->too long?
    ros::Rate loop_rate(1/3.0);

    std::vector<std::thread> threads;

    while(ros::ok()){ 
        threads.clear();
        connectedIPs.clear();

        /// Get the data to send to the Qt app
        std::string dataToSend = getDataToSend();
        //ROS_INFO("(ping_server) Data to send : %s", dataToSend.c_str());


        /// We create threads to ping every IP adress
        /// => threads reduce the required time to ping from ~12 sec to 2 sec
        serverMutex.lock();
        //ROS_INFO("(ping_server) Trying to ping everyone %lu", availableIPs.size());
        for(int i = 0; i < availableIPs.size(); ++i)
            threads.push_back(std::thread(pingIP, availableIPs.at(i), dataToSend));
        serverMutex.unlock();

        /// We join all the threads => we wait for all the threads to be done
        for(int i = 0; i < threads.size(); ++i)
            threads.at(i).join();

        //ROS_INFO("(ping_server) Done pinging everyone");

        updateIP();

        loop_rate.sleep();        
    }
}

//update old IP list with the now-connected IPs
void updateIP(){
    /// We check the oldIPs vs the new connected IP so we now which one disconnected
    std::vector<int> toRemove;
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
                ROS_WARN("The ip %s disconnected", oldIPs.at(i).first.c_str());
                std_msgs::String msg;
                msg.data = oldIPs.at(i).first;
                disco_pub.publish(msg);
                toRemove.push_back(i);
            } 
            else
                ROS_WARN("Could not ping IP %s for %d time(s)", oldIPs.at(i).first.c_str(), oldIPs.at(i).second);
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
}

/**
 * Try to connect to the given IP, if can, then read OK, then send some data
 */
void pingIP(std::string ip, std::string dataToSend){
    //ROS_INFO("(ping_server) Called pingIP : %s", ip.c_str());

    timeout::tcp::Client client;
    try {
        /// Try to connect to the remote Qt app
        //tx??//close client connection after timeout
        client.connect(ip, "6000", boost::posix_time::seconds(2));
        /// If we succesfully connect to the server, then the server is supposed to send us "OK"
        std::string read = client.read_some();
        if(read.compare("OK") == 0){
            /// Send the required data
            client.write_line(dataToSend, boost::posix_time::seconds(2));
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


/**
 * Check all the IP addresses we can find on the local network and put them in an array
 * Usually take 5 to 10 secs
 */
void checkNewServers(void){
    /// We check for new IP addresses every 10 secs
    ros::Rate loop_rate(0.1);

    while(ros::ok()){ 
        //ros::spinOnce();

        //ROS_INFO("(ping_server) Refreshing the list of potential servers");
        /// Script which will check all the IP on the local network and put them in a file
        const std::string ping_script = "sudo sh " + pingFile + " " + ipsFile;
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

        ROS_INFO("(ping_server) Potential servers :%lu, Connected IPs :%lu", availableIPs.size(),connectedIPs.size());
        loop_rate.sleep();        
    }
}

void mySigintHandler(int sig)
{   
    ros::shutdown();
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "ping_server");
    ros::NodeHandle n;

    signal(SIGINT, mySigintHandler);
    
    n.param<bool>("simulation", simulation, false);
    ROS_INFO("(Command system) simulation : %d", simulation);

    getGobotStatusSrv = n.serviceClient<gobot_msg_srv::GetGobotStatus>("/gobot_status/get_gobot_status");
    
    //Startup begin
    ROS_INFO("(ping_server) Waiting for Robot finding initial pose...");
    getGobotStatusSrv.waitForExistence(ros::Duration(30.0));
    getGobotStatusSrv.call(get_gobot_status);
    while((get_gobot_status.response.status!=-1 || get_gobot_status.response.text!="FOUND_POSE") && ros::ok() && !simulation){
        getGobotStatusSrv.call(get_gobot_status);
        ros::Duration(0.2).sleep();
    }
    ROS_INFO("(ping_server) Robot finding initial pose is ready.");
    //Startup end

    disco_pub = n.advertise<std_msgs::String>("/gobot_software/server_disconnected", 10);

    /// We get the path to the file with all the ips
    if(n.hasParam("ips_file"))
        n.getParam("ips_file", ipsFile);
    else {
        ROS_ERROR("(ping_server) The parameter <ips_file> does not exist");
        return -1;
    }

    /// We get the path to the script to retrieve all the ips
    if(n.hasParam("ping_file"))
        n.getParam("ping_file", pingFile);
    else {
        ROS_ERROR("(ping_server) The parameter <ping_file> does not exist");
        return -1;
    }

    /// Thread which will get an array of potential servers
    std::thread t1(checkNewServers);

    ROS_INFO("(ping_server) checkNewServers thread launched");

    /// We try to ping all the available IPs
    std::thread t2(pingAllIPs);

    ros::spin();

    return 0;
}