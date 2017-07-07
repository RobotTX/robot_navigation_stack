#include <ros/ros.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <std_srvs/Empty.h>
#include <gobot_base/GetEncoders.h>
#include <gobot_base/SetSpeeds.h>
#include "serial/serial.h"

std::mutex mtx;
serial::Serial serialConnection;

/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0){
    std::vector<uint8_t> buff;

    if(serialConnection.isOpen()){
        /// Lock the mutex so no one can write at the same time
        mtx.lock();

        /// Send bytes to the MD49
        size_t bytes_wrote = serialConnection.write(toWrite);

        /// Read any byte that we are expecting
        if(bytesToRead > 0)
            serialConnection.read(buff, bytesToRead);
        
        /// Unlock the mutex
        mtx.unlock();

    } else {
        std::cout << "(wheels::writeAndRead) The serial connection is not opened, something is wrong" << std::endl;
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

    std::cout << "(wheels::initSerial) MD49 port : " << port << std::endl;

    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
    serialConnection.setBaudrate(9600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();

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
    uint8_t leftSpeed = req.directionL.compare("F") == 0 ? 128 - req.velocityL : 128 + req.velocityL;
    uint8_t rightSpeed = req.directionR.compare("F") == 0 ? 128 - req.velocityR : 128 + req.velocityR;

    std::cout << "(wheels::setSpeeds) Data : " << req.directionL << " " << (int) req.velocityL << " " << req.directionR << " " << (int) req.velocityR << std::endl;
    
    writeAndRead(std::vector<uint8_t>({0x00, 0x31, leftSpeed, 0x00, 0x32, rightSpeed}));

    return true;
}

/// Get the encoders position
bool getEncoders(gobot_base::GetEncoders::Request &req, gobot_base::GetEncoders::Response &res){
    std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

    if(encoders.size() == 8){
        res.leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
        res.rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
        return true;
    } else {
        std::cout << "(wheels::getEncoders) Got the wrong number of encoders data : " << encoders.size() << std::endl;
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

    while(ros::ok()){
        std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

        if(encoders.size() == 8){
            int32_t leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
            int32_t rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
            std::cout << "(wheels::testEncoders) " << leftEncoder << " and " << rightEncoder << std::endl;
        } else
            std::cout << "(wheels::testEncoders) Got the wrong number of encoders data : " << encoders.size() << std::endl;
    }

    return true;
}

/// Get the encoders and print the difference between the new and previous encoder every 1 sec
bool testEncoders2(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ros::Rate r(1);
    int32_t last_leftEncoder = 0;
    int32_t last_rightEncoder = 0;

    while(ros::ok()){
        std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

        if(encoders.size() == 8){
            int32_t leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
            int32_t rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
            
            std::cout << "(wheels::testEncoders2) " << leftEncoder - last_leftEncoder << " and " << rightEncoder - last_rightEncoder << std::endl;
    
            last_leftEncoder = leftEncoder;
            last_rightEncoder = rightEncoder;

        } else
            std::cout << "(wheels::testEncoders2) Got the wrong number of encoders data : " << encoders.size() << std::endl;
        r.sleep();
    }

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

        ros::spin();
    } else
        std::cout << "(wheels::main) Could not open the serial communication" << std::endl;

    return 0;
}
