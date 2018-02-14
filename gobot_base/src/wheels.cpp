#include <gobot_base/wheels.hpp>

bool testEncoder = false;
int leftSpeed_=128, rightSpeed_=128;

ros::ServiceServer setSpeedsSrv,getEncodersSrv,resetEncodersSrv,initialMotorSrv;

std::mutex serialMutex;
std::string MD49device;
serial::Serial serialConnection;

/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead){
    std::vector<uint8_t> buff;
    
    /// Lock the mutex so no one can write at the same time
    serialMutex.lock();
    if(serialConnection.isOpen()){
        try{
            /// Send bytes to the MD49
            size_t bytes_wrote = serialConnection.write(toWrite);
            /// Read any byte that we are expecting
            if(bytesToRead > 0)
                serialConnection.read(buff, bytesToRead);

            //serialConnection.flush();
        } catch (std::exception& e) {
		    ROS_ERROR("(Wheels) exception : %s", e.what());
	    }
    } 
    else {
        ROS_WARN("(wheels::writeAndRead) The serial connection is not opened, something is wrong");
    }

    /// Unlock the mutex
    serialMutex.unlock();

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

bool getSpeeds(gobot_msg_srv::GetIntArray::Request &req, gobot_msg_srv::GetIntArray::Response &res){
    res.data.push_back(leftSpeed_);
    res.data.push_back(rightSpeed_);
    
    return true;
}


/// Set the speed, 0 (full reverse)  128 (stop)   255 (full forward)
//tx//('B',127)=0, ('F/B',0)=128,('F',127)=255
bool setSpeeds(gobot_msg_srv::SetSpeeds::Request &req, gobot_msg_srv::SetSpeeds::Response &res){
    if(req.velocityL <= 127 && req.velocityR <= 127){
        //x=condition?x1:x2   
        //condition=true,x=x1; condition=false,x=x2.
        //for larger dimension gobot
        //uint8_t leftSpeed = req.directionL.compare("F") == 0 ? 128 - req.velocityL : 128 + req.velocityL;
        //uint8_t rightSpeed = req.directionR.compare("F") == 0 ? 128 - req.velocityR : 128 + req.velocityR;

        uint8_t leftSpeed = req.directionL.compare("F") == 0 ? 128 + req.velocityL : 128 - req.velocityL;
        uint8_t rightSpeed = req.directionR.compare("F") == 0 ? 128 + req.velocityR : 128 - req.velocityR;

        leftSpeed_ = leftSpeed;
        rightSpeed_ = rightSpeed;
        //ROS_INFO("(wheels::setSpeeds) Data : %s %d %s %d", req.directionL.c_str(), (int) req.velocityL, req.directionR.c_str(), (int) req.velocityR);
        
        writeAndRead(std::vector<uint8_t>({0x00, 0x31, leftSpeed, 0x00, 0x32, rightSpeed}));

        return true;
    } 
    else 
        return false;
}

/// Get the encoders position
bool getEncoders(gobot_msg_srv::GetEncoders::Request &req, gobot_msg_srv::GetEncoders::Response &res){
    std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

    if(encoders.size() == 8){
        res.leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
        res.rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
        if(testEncoder){
            ROS_INFO("(wheels::testEncoders) %d and %d", res.leftEncoder, res.rightEncoder);
        }
        return true;
    } 
    else{
        ROS_WARN("(wheels::getEncoders) Got the wrong number of encoders data : %lu", encoders.size());
        return false;
    }
}

/// Set the encoders to 0
bool resetEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    writeAndRead(std::vector<uint8_t>({0x00, 0x35}));
    return true;
}

bool initialMotor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    serialMutex.lock();
    serialConnection.close();
    serialMutex.unlock();
    return initSerial();
}


/// Just to test, we get the encoders and print them
bool testEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    testEncoder = true;
    return true;
}


bool stopTests(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    testEncoder = false;
    return true;
}

bool motorReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return true;
}

bool initSerial() {
    /// Get the port in which our device is connected
    std::string port = MD49device;
    if(serialConnection.isOpen()){
        serialConnection.close();
    }
    ROS_INFO("(wheels::initSerial) MD49 port : %s", port.c_str());

    serialMutex.lock();
    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
    //Send 1200 bytes per second
    serialConnection.setBaudrate(9600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();


    if(serialConnection.isOpen()){
        /// First 3 bytes : set the mode (see MD49 documentation)
        /// then 2 bytes to disable the 2 sec timeout
        /// then 3 bytes to set the acceleration steps (1-10)
        serialConnection.write(std::vector<uint8_t>({0x00, 0x34, 0x00,0x00, 0x38,0x00, 0x33, 0x01}));
        ///reset Encoder
        serialConnection.write(std::vector<uint8_t>({0x00, 0x35}));
        ROS_INFO("(wheels::initSerial) Established connection to MD49.");
        serialMutex.unlock();
        return true;
    } 
    else{
        serialMutex.unlock();
        return false;
    }
}


void mySigintHandler(int sig)
{   
    resetEncodersSrv.shutdown();
    getEncodersSrv.shutdown();
    setSpeedsSrv.shutdown();
    initialMotorSrv.shutdown();

    serialMutex.lock();
    try{
        //set speed to 0 when shutdown
        serialConnection.write(std::vector<uint8_t>({0x00, 0x31, 0x80, 0x00, 0x32, 0x80}));
        serialConnection.close();
	} catch (std::exception& e) {
		ROS_ERROR("(Wheels) Shutdown exception : %s", e.what());
	}
    serialMutex.unlock();
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wheels");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    nh.getParam("MD49device", MD49device);

    if(initSerial()){
        setSpeedsSrv = nh.advertiseService("/gobot_motor/setSpeeds", setSpeeds);
        getEncodersSrv = nh.advertiseService("/gobot_motor/getEncoders", getEncoders);
        resetEncodersSrv = nh.advertiseService("/gobot_motor/resetEncoders", resetEncoders);
        initialMotorSrv = nh.advertiseService("/gobot_motor/initialMotor", initialMotor);
        ros::ServiceServer testEncodersSrv = nh.advertiseService("/gobot_test/testEncoders", testEncoders);
        ros::ServiceServer stopTestsSrv = nh.advertiseService("/gobot_test/stopTestEncoder", stopTests);
        ros::ServiceServer getSpeedsSrv = nh.advertiseService("/gobot_motor/getSpeeds", getSpeeds);

        //Startup begin
        ros::service::waitForService("/gobot_status/set_gobot_status", ros::Duration(60.0));
        ros::ServiceServer motorReadySrv = nh.advertiseService("/gobot_startup/motor_ready", motorReadySrvCallback);
        //Startup end
        
        ros::spin();
    } else
        ROS_INFO("(wheels::main) Could not open the serial communication");

    return 0;
}
