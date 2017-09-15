#include <gobot_base/wheels.hpp>

std::mutex serialMutex;
serial::Serial serialConnection;
bool test = false;

/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead){
    std::vector<uint8_t> buff;

    if(serialConnection.isOpen()){
        /// Lock the mutex so no one can write at the same time
        serialMutex.lock();

        /// Send bytes to the MD49
        size_t bytes_wrote = serialConnection.write(toWrite);

        /// Read any byte that we are expecting
        if(bytesToRead > 0)
            serialConnection.read(buff, bytesToRead);
        
        /// Unlock the mutex
        serialMutex.unlock();

    } else {
        ROS_INFO("(wheels::writeAndRead) The serial connection is not opened, something is wrong");
    }

    return buff;
}

/// get the output of the given system command
std::string getStdoutFromCommand(std::string cmd) {
    std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}

bool initSerial() {
    /// Get the port in which our device is connected
    std::string deviceNode("2-3.3:1.1");
    std::string output = getStdoutFromCommand("ls -l /sys/class/tty/ttyUSB*");
    std::string port = "/dev" + output.substr(output.find(deviceNode) + deviceNode.size(), 8);

    ROS_INFO("(wheels::initSerial) MD49 port : %s", port.c_str());

    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
    serialConnection.setBaudrate(9600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();

    ROS_INFO("(wheels::initSerial) MD49 serial communication : %d", serialConnection.isOpen());

    if(serialConnection.isOpen()){
        /// First 3 bytes : set the mode (see MD49 documentation)
        /// then 2 bytes to disable the 2 sec timeout
        /// then 3 bytes to set the acceleration steps
        writeAndRead(std::vector<uint8_t>({0x00, 0x34, 0x00, 
            0x00, 0x38, 
            0x00, 0x33, 0x01}));

        return true;
    } else
        return false;
}

/// Set the speed, 0 (full reverse)  128 (stop)   255 (full forward)
bool setSpeeds(gobot_base::SetSpeeds::Request &req, gobot_base::SetSpeeds::Response &res){
    if(req.velocityL <= 127 && req.velocityL <= 127){
        uint8_t leftSpeed = req.directionL.compare("F") == 0 ? 128 - req.velocityL : 128 + req.velocityL;
        uint8_t rightSpeed = req.directionR.compare("F") == 0 ? 128 - req.velocityR : 128 + req.velocityR;

        //ROS_INFO("(wheels::setSpeeds) Data : %s %d %s %d", req.directionL.c_str(), (int) req.velocityL, req.directionR.c_str(), (int) req.velocityR);
        
        writeAndRead(std::vector<uint8_t>({0x00, 0x31, leftSpeed, 0x00, 0x32, rightSpeed}));

        return true;
    } else 
        return false;
}

/// Get the encoders position
bool getEncoders(gobot_base::GetEncoders::Request &req, gobot_base::GetEncoders::Response &res){
    std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

    if(encoders.size() == 8){
        res.leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
        res.rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
        return true;
    } else {
        ROS_INFO("(wheels::getEncoders) Got the wrong number of encoders data : %lu", encoders.size());
        return false;
    }
}

/// Set the encoders to 0
bool resetEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    writeAndRead(std::vector<uint8_t>({0x00, 0x35}));

    return true;
}

/// Just to test, we get the encoders and print them
bool testEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    test = true;

    std::thread([](){
        while(ros::ok() && test){
            std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

            if(encoders.size() == 8){
                int32_t leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
                int32_t rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
                ROS_INFO("(wheels::testEncoders) %d and %d", leftEncoder, rightEncoder);
            } else
                ROS_INFO("(wheels::testEncoders) Got the wrong number of encoders data : %lu", encoders.size());
        }
    }).detach();

    return true;
}

/// Get the encoders and print the difference between the new and previous encoder every 1 sec
bool testEncoders2(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    test = true;

    std::thread([](){
        int32_t last_leftEncoder = 0;
        int32_t last_rightEncoder = 0;
        ros::Rate r(1);

        while(ros::ok() && test){
            std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

            if(encoders.size() == 8){
                int32_t leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
                int32_t rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
                
                ROS_INFO("(wheels::testEncoders2) %d and %d", leftEncoder - last_leftEncoder, rightEncoder - last_rightEncoder);
        
                last_leftEncoder = leftEncoder;
                last_rightEncoder = rightEncoder;

            } else
                ROS_INFO("(wheels::testEncoders2) Got the wrong number of encoders data : %lu", encoders.size());
            r.sleep();
        }
    }).detach();

    return true;
}

bool stopTests(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    test = false;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wheels");

    if(initSerial()){
        ros::NodeHandle nh;

        ros::ServiceServer setSpeedsSrv = nh.advertiseService("setSpeeds", setSpeeds);
        ros::ServiceServer getEncodersSrv = nh.advertiseService("getEncoders", getEncoders);
        ros::ServiceServer resetEncodersSrv = nh.advertiseService("resetEncoders", resetEncoders);
        ros::ServiceServer testEncodersSrv = nh.advertiseService("testEncoders", testEncoders);
        ros::ServiceServer testEncodersSrv2 = nh.advertiseService("testEncoders2", testEncoders2);
        ros::ServiceServer stopTestsSrv = nh.advertiseService("stopTests", stopTests);

        ros::spin();
    } else
        ROS_INFO("(wheels::main) Could not open the serial communication");

    return 0;
}
