#ifndef PING_SERVER_HPP
#define PING_SERVER_HPP

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <gobot_software/timeout_blocking_tcp_client.h>
#include <std_msgs/String.h>
#include <gobot_msg_srv/StringArrayMsg.h>
#include <gobot_msg_srv/GetString.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <string>
#include <mutex>
#include <thread>

#define PING_THRESHOLD 5

using boost::asio::ip::tcp;

class PingServerClass {
    public:
        PingServerClass(){
            ros::NodeHandle nh;
            nh.getParam("status_update", STATUS_UPDATE);

            disServersSrv_ = nh.advertiseService("/gobot_software/disconnet_servers", &PingServerClass::disServersSrvCallback,this);

            disco_pub_ = nh.advertise<std_msgs::String>("/gobot_software/server_disconnected", 1);

            host_pub_ = nh.advertise<gobot_msg_srv::StringArrayMsg>("/gobot_software/connected_servers", 1);

            update_sub_ = nh.subscribe("/gobot_status/update_information", 10, &PingServerClass::updateInfoCallback,this);

            servers_sub_ = nh.subscribe("/gobot_software/online_servers", 10, &PingServerClass::updateServersCallback,this);

            ping_thread_ = new boost::thread(&PingServerClass::PingThread, this);
        }


        ~PingServerClass(){
            update_sub_.shutdown();
            servers_sub_.shutdown();
            disServersSrv_.shutdown();
            
            ping_thread_->interrupt();
            ping_thread_->join();
            delete ping_thread_;

            for(int i=0;i<oldIPs_.size();i++){
                std_msgs::String msg;
                msg.data = oldIPs_.at(i).first;
                disco_pub_.publish(msg);
            }
        }


        /**
        * Service callback function to disconnect all hosts
        **/
        bool disServersSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
            ipMutex.lock();
            //disconnect all sockets when closed
            for(int i=0;i<oldIPs_.size();i++){
                std_msgs::String msg;
                msg.data = oldIPs_.at(i).first;
                disco_pub_.publish(msg);
            }
            oldIPs_.clear();
            ipMutex.unlock();

            //clear all IPs
            serverMutex.lock();
            availableIPs_.clear();
            serverMutex.unlock();

            disco_time_ = ros::Time::now();

            return true;
        }


        /**
        * Will update servers' IP address in local network
        **/
        void updateServersCallback(const gobot_msg_srv::StringArrayMsg::ConstPtr& servers_msg){
            serverMutex.lock();
            availableIPs_.clear();
            for(int i=0;i<servers_msg->data.size();i++){
                availableIPs_.push_back(servers_msg->data[i]);
            }
            serverMutex.unlock();
        }


        /**
        * Will update gobot status when receive any command
        **/
        void updateInfoCallback(const std_msgs::String::ConstPtr& update_status){
            std::vector<std::thread> data_threads;
            ipMutex.lock();
            if(oldIPs_.size()>0 && (ros::Time::now()-disco_time_)>ros::Duration(STATUS_UPDATE)){
                for(int i = 0; i < oldIPs_.size(); ++i)
                    data_threads.push_back(std::thread(&PingServerClass::pingIP2, this, oldIPs_.at(i).first, update_status->data, 3.0));
            }
            ipMutex.unlock();
            
            if(data_threads.size()>0){
                ///we wait for all the threads to be done
                for(int i = 0; i < data_threads.size(); ++i)
                    data_threads.at(i).detach();  
                ROS_INFO("(PING_SERVERS) Update robot info to every connected server");
            }
        }


        /**
        * Will send robot status to every available server's IP, and connect the host if the feedback is correct
        **/
        void pingIP(std::string ip, std::string dataToSend, double sec){
            //ROS_INFO("(PING_SERVERS) Called pingIP : %s", ip.c_str());
            timeout::tcp::Client client;
            try {
                /// Try to connect to the remote Qt app
                client.connect(ip, "6000", boost::posix_time::seconds(sec));
                /// If we succesfully connect to the server, then the server is supposed to send us "OK\n"
                std::string read = client.read_line(boost::posix_time::seconds(sec));
                if(read.compare("OK") == 0){
                    /// Send the required data
                    client.write_line(dataToSend, boost::posix_time::seconds(sec));
                    //ROS_INFO("Done with this ip: %s",ip.c_str());
                    connectedMutex.lock();
                    connectedIPs_.push_back(ip);
                    connectedMutex.unlock();
                }
                
            } catch(std::exception& e) {
                //ROS_ERROR("(PING_SERVERS) error %s : %s", ip.c_str(), e.what());
            }
        }


        /**
        * Will send robot status to every connected host's IP
        **/
        void pingIP2(std::string ip, std::string dataToSend, double sec){
            timeout::tcp::Client client;
            try {
                client.connect(ip, "6000", boost::posix_time::seconds(sec));
                std::string read = client.read_line(boost::posix_time::seconds(sec));
                if(read.compare("OK") == 0){
                    client.write_line(dataToSend, boost::posix_time::seconds(sec));
                } 

            } catch(std::exception& e) {
                //ROS_ERROR("(PING_SERVERS) error %s : %s", ip.c_str(), e.what());
            }
        }


        /**
        * Periodically try to send message to available server's IP
        **/
        void PingThread(){
            disco_time_ = ros::Time::now();
            ros::Rate loop_rate(1/STATUS_UPDATE);
            while(ros::ok()){ 
                std::vector<std::thread> threads;
                connectedIPs_.clear();
                serverMutex.lock();
                if(availableIPs_.size()>0 && (ros::Time::now()-disco_time_)>ros::Duration(STATUS_UPDATE)){
                    //ROS_INFO("(PING_SERVERS) Data to send : %s", dataToSend.c_str());
                    /// We create threads to ping every IP adress
                    /// => threads reduce the required time to ping from ~12 sec to 2 sec
                    /// Get the data to send to the Qt app
                    for(int i = 0; i < availableIPs_.size(); ++i)
                        threads.push_back(std::thread(&PingServerClass::pingIP, this, availableIPs_.at(i), getDataToSend(), 3.0));
                }
                serverMutex.unlock();
                
                ros::Time cur_time = ros::Time::now();
                if(threads.size()>0){
                    /// We join all the threads => we wait for all the threads to be done
                    for(int i = 0; i < threads.size(); ++i)
                        threads.at(i).join();
                }
                //ROS_INFO("(PING_SERVERS) Use %.2f seconds to ping %zu servers",(ros::Time::now()-cur_time).toSec(),threads.size());
                updateIP();
                loop_rate.sleep();        
            }
        }

    private:
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
        * Update connected hosts' IP 
        **/
        void updateIP(){
            /// We check the oldIPs_ vs the new connected IP so we now which one disconnected
            std::vector<int> toRemove;
            ipMutex.lock();
            for(int i = 0; i < oldIPs_.size(); ++i){
                bool still_connected = false;
                for(int j = 0; j < connectedIPs_.size(); ++j){
                    if(oldIPs_.at(i).first.compare(connectedIPs_.at(j)) == 0){
                        still_connected = true;
                        break;
                    }
                }
                if(still_connected){
                    oldIPs_.at(i).second = 0;
                }
                else{
                    oldIPs_.at(i).second++;
                    if(oldIPs_.at(i).second >= PING_THRESHOLD){
                        /// Publish a message to the topic /gobot_software/server_disconnected with the IP address of the server which disconnected
                        ROS_WARN("(PING_SERVERS) The ip %s disconnected", oldIPs_.at(i).first.c_str());
                        std_msgs::String msg;
                        msg.data = oldIPs_.at(i).first;
                        disco_pub_.publish(msg);
                        toRemove.push_back(i);
                    } 
                }
            }

            /// We remove all the disconnected IPs from the oldIPs_ vector
            for(int i = toRemove.size()-1; i >= 0; i--)
                oldIPs_.erase(oldIPs_.begin() + toRemove.at(i));

            /// We had the newly connectedIPs_ to the oldIPs_ vector
            for(int i = 0; i < connectedIPs_.size(); ++i){
                bool hasIP = false;
                for(int j = 0; j < oldIPs_.size(); ++j){
                    if(connectedIPs_.at(i).compare(oldIPs_.at(j).first) == 0){
                        hasIP = true;
                        break;
                    }
                }
                if(!hasIP)
                    oldIPs_.push_back(std::pair<std::string, int>(connectedIPs_.at(i), 0));
            }

            gobot_msg_srv::StringArrayMsg host_ip_msg;
            for(int i = 0; i < oldIPs_.size(); ++i){
                host_ip_msg.data.push_back(oldIPs_.at(i).first);
            }
            host_pub_.publish(host_ip_msg);
            //ROS_INFO("(PING_SERVERS) Done update servers IP. %zu servers connected", oldIPs_.size());
            ipMutex.unlock();
        }

        double STATUS_UPDATE;
        std::vector<std::string> availableIPs_, connectedIPs_;
        std::vector<std::pair<std::string, int>> oldIPs_;
        std::mutex serverMutex, connectedMutex, ipMutex;

        boost::thread *ping_thread_;

        ros::ServiceServer disServersSrv_;
        ros::Publisher disco_pub_, host_pub_;
        ros::Subscriber servers_sub_, update_sub_;
        ros::Time disco_time_;

};

#endif
