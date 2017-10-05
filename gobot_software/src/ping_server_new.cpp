#include "gobot_software/ping_server_new.hpp"

static const std::string sep = std::string(1, 31);

std::string pingFile;
std::string ipsFile;
std::string nameFile;
std::string pathStageFile;

bool chargingFlag = false;
int batteryLevel = 50;

std::vector<std::string> availableIPs;
std::vector<std::string> oldIPs;
std::vector<std::string> connectedIPs;
std::mutex serverMutex;
std::mutex connectedMutex;

ros::Publisher disco_pub;

/**
 * Create the string with information to connect to the Qt app
 * hostname + sep + stage + sep + batteryLevel + sep + chargingFlag + sep + dockStatus
 */
std::string getDataToSend(void){
    /// Retrieves the hostname
    std::ifstream ifs(nameFile, std::ifstream::in);
    std::string hostname;
    if(ifs){
        ifs >> hostname;
        ifs.close();
    } else 
        ROS_ERROR("(ping_server) could not open file %s", nameFile.c_str());

    if(hostname.empty())
        hostname = "Default Name";

    /// Retrieve the docking status
    gobot_msg_srv::GetDockStatus _dockStatus;
    int dockStatus = 0;
    if(ros::service::call("getDockStatus", _dockStatus))
        dockStatus = _dockStatus.response.status;
    else
        ROS_ERROR("(ping_server) could not call getDockStatus service");

    /// Retrieves the path stage
    int stage(0);
    std::ifstream ifsStage(pathStageFile, std::ifstream::in);
    if(ifsStage){
        ifsStage >> stage;
        ifsStage.close();
    } else
        ROS_ERROR("(ping_server) could not open file to retrieve path stage");

    /// Form the string to send to the Qt app
    return hostname + sep + std::to_string(stage) + sep + std::to_string(batteryLevel) + sep + std::to_string(chargingFlag) + sep + std::to_string(dockStatus);
}

/**
 * Will ping all available IP addresses and send a message when we disconnect to one
 */
void pingAllIPs(void){
    /// Ping all servers every 3 secs
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
        ROS_INFO("(ping_server) Trying to ping everyone %lu", availableIPs.size());
        for(int i = 0; i < availableIPs.size(); ++i)
            threads.push_back(std::thread(pingIP, availableIPs.at(i), dataToSend));
        serverMutex.unlock();

        /// We join all the threads => we wait for all the threads to be done
        for(int i = 0; i < threads.size(); ++i)
            threads.at(i).join();

        ROS_INFO("(ping_server) Done pinging everyone");

        /// We check the oldIPs vs the new connected IP so we now which one disconnected
        ROS_INFO("(ping_server) Connected IPs : %lu", connectedIPs.size());
        for(int i = 0; i < oldIPs.size(); ++i){
            bool still_connected = false;
            for(int j = 0; j < connectedIPs.size(); ++j)
                if(oldIPs.at(i).compare(connectedIPs.at(j)) == 0)
                    still_connected = true;
            if(!still_connected){
                /// Publish a message to the topic /server_disconnected with the IP address of the server which disconnected
                ROS_WARN("The ip %s disconnected", oldIPs.at(i).c_str());
                std_msgs::String msg;
                msg.data = oldIPs.at(i);
                disco_pub.publish(msg);
            }
        }

        oldIPs = connectedIPs;

        loop_rate.sleep();        
    }
}

/**
 * Try to connect to the given IP, if can, then read OK, then send some data
 */
void pingIP(std::string ip, std::string dataToSend){
    //ROS_INFO("(ping_server) Called pingIP : %s", ip.c_str());

    try {
        timeout::tcp::Client client;
        /// Try to connect to the remote Qt app
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
            ROS_INFO("(ping_server) Connected to %s", ip.c_str());
        } else
            ROS_ERROR("(ping_server) read %lu bytes : %s", read.size(), read.c_str());
    } catch(std::exception& e) {
        //ROS_ERROR("(ping_server) error %s : %s", ip.c_str(), e.what());
    }
}

/**
 * Update the battery informations we need to send to the Qt app
 */
void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    chargingFlag = batteryInfo->ChargingFlag;
    if(batteryInfo->FullCapacity != 0)
        batteryLevel = batteryInfo->BatteryStatus;
}

/**
 * Check all the IP addresses we can find on the local network and put them in an array
 * Usually take 5 to 10 secs
 */
void checkNewServers(void){
    /// We check for new IP addresses every 10 secs
    ros::Rate loop_rate(0.1);

    while(ros::ok()){ 
        ros::spinOnce();

        ROS_INFO("(ping_server) Refreshing the list of potential servers");
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
            while(std::getline(ifs, currentIP))
                availableIPs.push_back(currentIP);

            serverMutex.unlock();
        }


        ROS_INFO("(ping_server) Done refreshing the list of potential servers : %lu servers found", availableIPs.size());

        loop_rate.sleep();        
    }
}


int main(int argc, char* argv[]){

    ros::init(argc, argv, "ping_server");
    ros::NodeHandle n;

    ros::Subscriber batterySub = n.subscribe("/battery_topic", 1, newBatteryInfo);

    disco_pub = n.advertise<std_msgs::String>("server_disconnected", 10);

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

    /// File containing the name of the robot
    if(n.hasParam("robot_name_file"))
        n.getParam("robot_name_file", nameFile);
    else {
        ROS_ERROR("(ping_server) The parameter <robot_name_file> does not exist");
        return -1;
    }

    /// File containing the stage of the path
    if(n.hasParam("path_stage_file"))
        n.getParam("path_stage_file", pathStageFile);
    else
        ROS_INFO("(ping_server) parameter <path_stage_file> does not exist");


    ros::Publisher publisher = n.advertise<std_msgs::String>("server_disconnected", 10);

    /// Thread which will get an array of potential servers
    std::thread t1(checkNewServers);

    ROS_INFO("(ping_server) checkNewServers thread launched");

    /// We try to ping all the available IPs
    std::thread t2(pingAllIPs);

    ros::spin();

    return 0;
}