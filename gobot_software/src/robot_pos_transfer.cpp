#include "gobot_software/robot_pos_transfer.hpp"

#define PORT_ROBOT_POS 4001

using boost::asio::ip::tcp;
ros::Timer timer1, timer2;

std::mutex socketsMutex;
std::map<std::string, boost::shared_ptr<tcp::socket>> sockets;
std::string robot_string;
std::string lastPoseFile;
bool found_pose = false;

double last_pos_x=0.0,last_pos_y=0.0,last_pos_yaw=0.0,last_ang_x=0.0,last_ang_y=0.0,last_ang_z=0.0,last_ang_w=1.0;

void sendRobotPosTimer(const ros::TimerEvent&){
    if(sockets.size() > 0){
        socketsMutex.lock();
        /// We send the position of the robot to every Qt app
        for(auto const &elem : sockets){
            try {
                boost::asio::write(*(elem.second), boost::asio::buffer(robot_string, robot_string.length()));
            } 
            catch (std::exception& e) {
                //can not send pose to the ip
                //ROS_ERROR("(POS_TRANSFER) Exception %s : %s", elem.first.c_str(), e.what());
            }
        }
        socketsMutex.unlock(); 
    }
}

void saveRobotPosTimer(const ros::TimerEvent&){
    if(found_pose){
        //save pose to local file
        std::ofstream ofs(lastPoseFile, std::ofstream::out | std::ofstream::trunc);
        if(ofs.is_open()){
            ofs << last_pos_x << " " << last_pos_y << " " << last_ang_x <<" "<< last_ang_y <<" "<< last_ang_z <<" "<< last_ang_w;
            ofs.close();
        } 
    }
}

void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg){
    if(ros::service::exists("/gobot_startup/pose_ready",false)){
        double yaw = tf::getYaw(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
        robot_string = std::to_string(msg->position.x) + " " + std::to_string(msg->position.y) + " " + std::to_string(yaw) + " ";
        last_pos_x = msg->position.x;
        last_pos_y = msg->position.y;
        last_ang_x = msg->orientation.x;
        last_ang_y = msg->orientation.y;
        last_ang_z = msg->orientation.z;
        last_ang_w = msg->orientation.w;
        found_pose = true;
    }
    else{
        found_pose = false;
    }
}

/*********************************** CONNECTION FUNCTIONS ***********************************/

void server(void){
    boost::asio::io_service io_service;
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), PORT_ROBOT_POS));
    while(ros::ok()) {

        boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(io_service));
        /// We wait for someone to connect
        a.accept(*sock);

        /// Got a new connection so we had it to our array of sockets
        std::string ip = sock->remote_endpoint().address().to_string();
        //ROS_INFO("(POS_TRANSFER) Command socket connected to %s", ip.c_str());
        socketsMutex.lock();

        if(sockets.find(ip)==sockets.end()){ //not find the ip
            sockets.insert(std::pair<std::string, boost::shared_ptr<tcp::socket>>(ip, sock));
        }
        else{   //ip exists in the list
            sockets.find(ip)->second->close();
            sockets.find(ip)->second = sock;
        }
        socketsMutex.unlock();
    }
}

/*********************************** DISCONNECTION FUNCTIONS ***********************************/

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    disconnect(msg->data);
}

void disconnect(const std::string ip){
    /// Close and remove the socket
    socketsMutex.lock();
    if(sockets.count(ip)){
        sockets.at(ip)->close();
        sockets.erase(ip);
        //ROS_WARN("(POS_TRANSFER) The ip %s just disconnected", ip.c_str());
    }
    socketsMutex.unlock();
}

/*********************************** SHUT DOWN ***********************************/
void mySigintHandler(int sig){ 
    timer2.stop();
    timer1.stop();
    if(found_pose){
        //save robot pose when node shuts down
        std::ofstream ofs(lastPoseFile, std::ofstream::out | std::ofstream::trunc);
        if(ofs.is_open()){
            ofs << last_pos_x << " " << last_pos_y << " " << last_ang_x <<" "<< last_ang_y <<" "<< last_ang_z <<" "<< last_ang_w;
            ofs.close();
        }
    }
    ros::shutdown();
}

/*********************************** MAIN ***********************************/

void initParams(){
    ros::NodeHandle nh;
    nh.getParam("last_pose_file", lastPoseFile);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "robot_pos_transfer");
	ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/network_ready", ros::Duration(120.0));
    //Startup end

    initParams();
    
    ros::Subscriber servers_disco_sub = n.subscribe("/gobot_software/server_disconnected", 1000, serverDisconnected);

    ros::Subscriber robot_pose_sub = n.subscribe("/robot_pose", 1, getRobotPos);

    //Periodically send robot pose to connected clients
    timer1 = n.createTimer(ros::Duration(0.5), sendRobotPosTimer);
    //Periodically save robot pose to local file
    timer2 = n.createTimer(ros::Duration(2.0), saveRobotPosTimer);

    std::thread t(server);
    ROS_INFO("(POS_TRANSFER) Ready to be launched.");

    ros::spin();

	return 0;
}

