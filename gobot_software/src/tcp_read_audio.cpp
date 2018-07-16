#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

#define NEW_AUDIO_PORT 4005

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

const int max_length = 1024;
const int ACK_NUM = 4;
std::vector<char> ACK_DATA({'!','@','#','$'});
int file_index = 0;
std::string folder_path = "/home/tx/catkin_ws/audio/";

void session(boost::shared_ptr<tcp::socket> sock){
    std::string ip = sock->remote_endpoint().address().to_string();
    long total_size = 0;
    std::vector<char> receive_msg;

    while(ros::ok() && sockets.count(ip)){
        std::cout<<"going to read data:"<<std::endl;
        char data[max_length];
        boost::system::error_code error;
        size_t data_size = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            std::cout <<boost::system::system_error(error).what() << std::endl; 
            return;
        } 
        else if (error){
            throw boost::system::system_error(error); // Some other error.
            return;
        }

        total_size += data_size;

        for(int i=0;i<data_size;i++)
            receive_msg.push_back(data[i]);
        
        std::cout<<"received: "<<data_size<<std::endl;
        //save mp3 file
        if(receive_msg.size()>ACK_NUM){
            bool correct_ack = true;
            for(int i=0; i<ACK_NUM; i++){
                if(receive_msg[total_size-ACK_NUM+i]!=ACK_DATA[ACK_NUM]){
                    correct_ack = false;
                    break;
                }
            }
            if(correct_ack){
                //delete ack
                receive_msg.erase(receive_msg.end()-ACK_NUM,receive_msg.end());

                std::ofstream desstr;
                desstr.open(folder_path+std::to_string(file_index)+".mp3", std::ios::out | std::ios::trunc | std::ios::binary);

                for(int i=0;i<receive_msg.size();i++)
                    desstr<<receive_msg[i];

                desstr.close();
                
                ROS_INFO("(READ_AUDIO) Received %d audio file, size: %zu.",file_index, receive_msg.size());
                total_size = 0;
                receive_msg.clear();
                file_index++;

                std::string message = "MP3 received!";
                //feedback to TCP if receiving same map
                boost::asio::write(*sock, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), error);
                if (error) 
                { 
                    std::cout <<boost::system::system_error(error).what() << std::endl; 
                } 
                return;
            }
        }
    }
}

/*********************************** CONNECTION FUNCTIONS ***********************************/
void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), NEW_AUDIO_PORT));
    while(ros::ok()) {

        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));
        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        ROS_INFO("(READ_AUDIO) Command socket connected to %s", ip.c_str());
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


bool fileIndexSrvCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    file_index = 0;
    std::string cmd = "sudo rm -f " + folder_path +"*";
    system(cmd.c_str());
    ROS_INFO("(READ_AUDIO) Clear file index and existing audio files");
    return true;
}

/*********************************** DISCONNECTION FUNCTIONS ***********************************/
void disconnect(const std::string ip){
    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(ip)){
        sockets.at(ip)->close();
        sockets.erase(ip);
        //ROS_WARN("(READ_AUDIO) The ip %s just disconnected", ip.c_str());
    }
    socketsMutex.unlock();
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    disconnect(msg->data);
}


/*********************************** MAIN ***********************************/
int main(int argc, char **argv){
	ros::init(argc, argv, "tcp_read_audio");
	ros::NodeHandle n;
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    //ros::Duration(1.0).sleep();
    //ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    //Startup end

    ros::Subscriber servers_disco_sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);
    ros::ServiceServer fileIndexSrv = n.advertiseService("/gobot_software/new_audio", fileIndexSrvCb);

    std::thread t(server);
    ROS_INFO("(READ_AUDIO) Ready to be launched.");

    ros::spin();

	return 0;
}

