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

#define NEW_AUDIO_PORT 5603

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;

static const std::string sep = std::string(1, 31);

const int max_length = 1024, ACK_NUM = 4;

int file_index = 0;
std::vector<char> ACK_DATA({'!','@','#','$'}), END_DATA({'!','!','!','!'});

std::string audio_folder = "/home/tx/audio/", feedback_msg = "done" + sep + "mp3";;

void session(boost::shared_ptr<tcp::socket> sock){
    std::string ip = sock->remote_endpoint().address().to_string();
    long data_size = 0;
    std::vector<char> audio_data;

    while(ros::ok() && sockets.count(ip)){
        char data[max_length];
        boost::system::error_code error;

        //wait for data
        size_t receive_size = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            std::cout <<boost::system::system_error(error).what() << std::endl; 
            return;
        } 
        else if (error){
            throw boost::system::system_error(error); // Some other error.
            return;
        }

        data_size += receive_size;

        for(int i=0;i<receive_size;i++)
            audio_data.push_back(data[i]);
        
        std::cout<<"receive size: "<<receive_size<<" total size: "<< data_size<<std::endl;
        //identify the end acknowledgement message
        if(audio_data.size()>ACK_NUM){
            bool correct_ack = true, correct_end = true;
            for(int i=1; i<=ACK_NUM; i++){
                if(!correct_ack && !correct_end)
                    break;
                //whether the message is the end of mp3 file
                if(audio_data[data_size-i]!=ACK_DATA[ACK_NUM-i] && correct_ack){
                    correct_ack = false;
                }
                //whether the message is the end of last mp3 file
                if(audio_data[data_size-i]!=END_DATA[ACK_NUM-i] && correct_end){
                    correct_end = false;
                }
            }
            //if it is the end of mp3 file
            if(correct_ack | correct_end){
                //delete ack message from the received data
                audio_data.erase(audio_data.end()-ACK_NUM,audio_data.end());

                //open the file that is going to save the mp3 data
                std::ofstream desstr;
                desstr.open(audio_folder+std::to_string(file_index)+".mp3", std::ios::out | std::ios::trunc | std::ios::binary);

                for(int i=0;i<audio_data.size();i++)
                    desstr<<audio_data[i];

                desstr.close();
                
                ROS_INFO("(READ_AUDIO) Received the %d audio file, size: %zu.",file_index+1, audio_data.size());
                data_size = 0;
                audio_data.clear();
                file_index++;

                //feedback to TCP if sth wrong
                boost::asio::write(*sock, boost::asio::buffer(feedback_msg, feedback_msg.length()), boost::asio::transfer_all(), error);
                if (error) 
                { 
                    std::cout <<boost::system::system_error(error).what() << std::endl; 
                }

                //close the session after completing reading the data of last mp3 file
                if(correct_end){ 
                    ROS_INFO("(READ_AUDIO) Received the last (%d) audio file, size: %zu. Close TCP session.",file_index+1, audio_data.size());
                    file_index = 0;
                    return;
                }
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
    std::string cmd = "sudo rm -f " + audio_folder +"*";
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
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    //Startup end
    if(n.hasParam("audio_folder"))
        n.getParam("audio_folder", audio_folder);

    ros::Subscriber servers_disco_sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);
    ros::ServiceServer fileIndexSrv = n.advertiseService("/gobot_software/clear_audio", fileIndexSrvCb);

    std::thread t(server);
    ROS_INFO("(READ_AUDIO) Ready to be launched.");

    ros::spin();

	return 0;
}

