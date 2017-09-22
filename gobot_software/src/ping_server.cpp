#include "gobot_software/ping_server.hpp"

static const std::string sep = std::string(1, 31);

bool chargingFlag = false;
int batteryLevel = 50;

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    chargingFlag = batteryInfo->ChargingFlag;
    if(batteryInfo->FullCapacity != 0)
        batteryLevel = batteryInfo->RemainCapacity / batteryInfo->FullCapacity * 100;
}


bool isServer(const std::string IP, const std::string ssid){

    ros::NodeHandle n;

    boost::asio::io_service io_service;

    tcp::resolver resolver(io_service);

    tcp::resolver::query query(IP);

    tcp::socket socket(io_service);

    if(socket.is_open())
        socket.close();

    try {
    
        socket.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(IP), 6000));

        char buffer[1024] = {0};

        boost::system::error_code error;
        size_t length = socket.read_some(boost::asio::buffer(buffer), error);

        if(std::string(buffer).compare("OK") == 0){

            // Retrieves the file that contains the hostname of the robot
            if(n.hasParam("robot_name_file")){
                std::string nameFile;
                n.getParam("robot_name_file", nameFile);

                // Retrieves the hostname
                std::ifstream ifs(nameFile, std::ifstream::in);
                std::string hostname;
                if(ifs){
                    getline(ifs, hostname);
                    ifs.close();
                } else 
                    ROS_INFO("(ping_server) could not open file %s", nameFile.c_str());

                if(hostname.empty())
                    hostname = "Default Name";

                gobot_msg_srv::GetDockStatus _dockStatus;
                int dockStatus = 0;
                if(ros::service::call("getDockStatus", _dockStatus))
                    dockStatus = _dockStatus.response.status;
                else
                    ROS_INFO("(ping_server) could not call getDockStatus service");

                // Retrieves the path stage
                int stage(0);
                std::string pathStageFile;
                if(n.hasParam("path_stage_file"))
                    n.getParam("path_stage_file", pathStageFile);
                else
                    ROS_INFO("(ping_server) parameter <path_stage_file> does not exist");

                std::ifstream ifsStage(pathStageFile, std::ifstream::in);
                if(ifsStage){
                    ifsStage >> stage;
                    ifsStage.close();
                } else
                    ROS_INFO("(ping_server) could not open file to retrieve path stage");

                // Sends everything to the application, '\' is used to as a delimiter, spaces cannot be used because of the ssid
                std::string info_to_send(hostname + sep + ssid + sep + std::to_string(stage) + sep + std::to_string(batteryLevel) + sep + std::to_string(chargingFlag) + sep + std::to_string(dockStatus));
                //ROS_INFO("(ping_server) sending %s", info_to_send.c_str());

                boost::system::error_code ignored_error;
                boost::asio::write(socket, boost::asio::buffer(info_to_send, info_to_send.length()), boost::asio::transfer_all(), ignored_error);
                return true;
            }
        } else
            ROS_INFO("(ping_server):: parameter <robot_name_file> does not exist");

            
        
    } catch(std::exception& e) {
        socket.close();
        ROS_INFO("(ping_server) error %s", e.what());
    }
    socket.close();
    return false;
}

bool checkIPList(const std::string ssid){
    ROS_INFO("(ping_server) checking IP list");
    ros::NodeHandle n;
    std::string IPListFile;
    if(n.hasParam("ips_file"))
        n.getParam("ips_file", IPListFile);
    else
        ROS_INFO("(ping_server) parameter <ips_file> does not exist");

    std::ifstream ifs(IPListFile, std::ifstream::in);
    if(ifs){
        std::string currentIP;
        while(std::getline(ifs, currentIP)){
            bool found = isServer(currentIP, ssid);
            if(found){
                std::string serverFile;
                if(n.hasParam("server_file"))
                    n.getParam("server_file", serverFile);
                else
                    ROS_INFO("(ping_server) parameter <server_file> does not exist");

                std::ofstream ofs(serverFile, std::ofstream::out | std::ofstream::trunc);
                if(ofs)
                    ofs << currentIP;
                else
                    ROS_INFO("(ping_server) could not open file to store server");
                return true;
            }
        }
    } else
        ROS_INFO("(ping_server) could not open IP list file");

    return false;
}

void client(ros::Publisher& publisher, const bool checkOldServer, const std::string ssid, const std::string serverFile, const std::string pingFile){

    bool found(false);
    ros::NodeHandle n;

    if(checkOldServer){
        found = false;
        std::ifstream ifs(serverFile, std::ifstream::in);
        if(ifs){
            std::string IP;
            ifs >> IP;
            found = isServer(IP, ssid);
            if(!found && ros::ok()){ 
                ROS_INFO("(ping_server) is scanning for a new server");
                std_msgs::String msg;
                msg.data = "disconnected";
                publisher.publish(msg);
                const std::string ping_script = "sudo sh " + pingFile;
                system(ping_script.c_str());
                client(publisher, false, ssid, serverFile, pingFile);
            }
        } else 
            ROS_INFO("(ping_server) could not open server file");
    }

    else {
        found = checkIPList(ssid);
        if(!found) 
            ROS_INFO("(ping_server) server not found");
    }
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "ping_server");
    ros::NodeHandle n;

    ros::Subscriber batterySub = n.subscribe("/battery_topic", 1, newBatteryInfo);

    std::string serverFile;
    if(n.hasParam("server_file"))
        n.getParam("server_file", serverFile);
    else 
        ROS_INFO("(ping_server):: The parameter <server_file> does not exist");

    std::string pingFile;
    if(n.hasParam("ping_file"))
        n.getParam("ping_file", pingFile);
    else
        ROS_INFO("(ping_server):: The parameter <ping_file> does not exist");

    ros::Publisher publisher = n.advertise<std_msgs::String>("server_disconnected", 10);

    ros::Rate loop_rate(1);

    FILE *in(0);
    char buff[512];
    // gets the ssid in a file
    if(!(in = popen("iwgetid -r", "r"))){
        ROS_INFO("(ping_server) could not get the ssid");
        return 1;
    }
    // retrieves the ssid from the file
    fgets(buff, sizeof(buff), in);
    std::string ssid(buff);

    // to get rid of \n at the end
    ssid = ssid.substr(0, ssid.size()-1);
    pclose(in);

    while(ros::ok()){ 
        ros::spinOnce();
        client(publisher, true, ssid, serverFile, pingFile);
        loop_rate.sleep();        
    }

    return 0;
}