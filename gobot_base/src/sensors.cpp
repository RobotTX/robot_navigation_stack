#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <gobot_base/SetSpeeds.h>
#include <gobot_base/SetSpeeds.h>
#include <gobot_base/BatteryMsg.h>
#include <gobot_base/BumperMsg.h>
#include <gobot_base/IrMsg.h>
#include <gobot_base/ProximityMsg.h>
#include <gobot_base/SonarMsg.h>
#include <gobot_base/WeightMsg.h>
#include <gobot_base/CliffMsg.h>
#include "serial/serial.h"


ros::Publisher bumper_pub;
ros::Publisher ir_pub;
ros::Publisher proximity_pub;
ros::Publisher sonar_pub;
ros::Publisher weight_pub;
ros::Publisher battery_pub;
ros::Publisher cliff_pub;
serial::Serial serialConnection;


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

void publishSensors(void){
    if(serialConnection.isOpen()){
        serialConnection.write(std::vector<uint8_t>({0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1}));

        std::vector<uint8_t> buff;
        serialConnection.read(buff, 47);

        //std::cout << "(sensors::publishSensors) Info : " << buff.size() << " " << (int) buff.at(0) << " " << (int) buff.at(1) << " " << (int) buff.at(2) << std::endl;

        /// check if the 16 is useful
        if(buff.size() == 47){
            /// First 3 bytes are the sensor address, the command and the data length so we can ignore it
            /// Then comes sonars data
            gobot_base::SonarMsg sonar_data;
            sonar_data.distance1 = buff.at(3) * 256 + buff.at(4);
            sonar_data.distance2 = buff.at(5) * 256 + buff.at(6);
            sonar_data.distance3 = buff.at(7) * 256 + buff.at(8);
            sonar_data.distance4 = buff.at(9) * 256 + buff.at(10);
            sonar_data.distance5 = buff.at(11) * 256 + buff.at(12);
            sonar_data.distance6 = buff.at(13) * 256 + buff.at(14);
            sonar_data.distance7 = buff.at(15) * 256 + buff.at(16);
            sonar_pub.publish(sonar_data);

            /// Bumpers data
            gobot_base::BumperMsg bumper_data;
            bumper_data.bumper1 = (buff.at(17) & 0b00000001) > 0;
            bumper_data.bumper2 = (buff.at(17) & 0b00000010) > 0;
            bumper_data.bumper3 = (buff.at(17) & 0b00000100) > 0;
            bumper_data.bumper4 = (buff.at(17) & 0b00001000) > 0;
            bumper_data.bumper5 = (buff.at(17) & 0b00010000) > 0;
            bumper_data.bumper6 = (buff.at(17) & 0b00100000) > 0;
            bumper_data.bumper7 = (buff.at(17) & 0b01000000) > 0;
            bumper_data.bumper8 = (buff.at(17) & 0b10000000) > 0;
            bumper_pub.publish(bumper_data);

            /// Ir signals
            gobot_base::IrMsg ir_data;
            ir_data.rearSignal = buff.at(18);
            ir_data.leftSignal = buff.at(19);
            ir_data.rightSignal = buff.at(20);
            ir_pub.publish(ir_data);

            /// Proximity sensors
            gobot_base::ProximityMsg proximity_data;
            proximity_data.signal1 = (buff.at(21) & 0b00000001) > 0;
            proximity_data.signal2 = (buff.at(21) & 0b00000010) > 0;
            proximity_pub.publish(proximity_data);

            /// Cliff sensors
            gobot_base::CliffMsg cliff_data;
            cliff_data.cliff1 = buff.at(22) * 256 + buff.at(23);
            cliff_data.cliff2 = buff.at(24) * 256 + buff.at(25);
            cliff_data.cliff3 = buff.at(26) * 256 + buff.at(27);
            cliff_data.cliff4 = buff.at(28) * 256 + buff.at(29);
            cliff_pub.publish(cliff_data);


            /// Battery data
            gobot_base::BatteryMsg battery_data;
            battery_data.BatteryStatus  = buff.at(31);
            battery_data.BatteryVoltage = buff.at(32) * 256 + buff.at(33);
            battery_data.ChargingCurrent = buff.at(34) * 256 + buff.at(35);
            battery_data.Temperature = buff.at(36) * 256 + buff.at(37);
            battery_data.RemainCapacity = (buff.at(38) * 256 + buff.at(39))/100;
            battery_data.FullCapacity = (buff.at(40) * 256 + buff.at(41))/100;
            battery_data.ChargingFlag = battery_data.ChargingCurrent > 500;
            battery_pub.publish(battery_data);

            /// Weight data
            gobot_base::WeightMsg weight_data;
            weight_data.weightInfo = buff.at(43) * 256 + buff.at(44);
            weight_pub.publish(weight_data);

            /// External button
            int32_t external_button = buff.at(45);
            /// TODO do whatever with we want with it

            /*
            std::cout << "Sonars : " << sonar_data.distance1 << " " << sonar_data.distance2 << " " << sonar_data.distance3 << " " << sonar_data.distance4 
            << " " << sonar_data.distance5 << " " << sonar_data.distance6 << " " << sonar_data.distance7 << std::endl;
            std::cout << "Bumpers : "  << (int) bumper_data.bumper1 << " " << (int) bumper_data.bumper2 << " " << (int) bumper_data.bumper3 << " " << (int) bumper_data.bumper4 << " "
            << (int) bumper_data.bumper5 << " " << (int) bumper_data.bumper6 << " " << (int) bumper_data.bumper7 << " " << (int) bumper_data.bumper8 << std::endl;
            std::cout << "Ir signals : " << (int) ir_data.rearSignal << " "  << (int) ir_data.leftSignal << " "  << (int) ir_data.rightSignal << std::endl;
            std::cout << "Proximity : " << (int) proximity_data.signal1 << " " << (int) proximity_data.signal2 << std::endl;
            std::cout << "Cliff : " << cliff_data.cliff1 << " " << cliff_data.cliff2 << " " << cliff_data.cliff3 << " " << cliff_data.cliff4 << std::endl;
            std::cout << "Battery : " << battery_data.BatteryStatus << " " << battery_data.BatteryVoltage << " " << battery_data.ChargingCurrent << " " << battery_data.Temperature 
            << " " << battery_data.RemainCapacity << " " << battery_data.FullCapacity << " " << battery_data.ChargingFlag << std::endl;
            std::cout << "Weight : " << weight_data.weightInfo << std::endl;
            std::cout << "External button : " << external_button << std::endl;
*/
            /// The last byte is the Frame Check Sum and is not used

        } else
            std::cout << "(sensors::publishSensors) Check buff size : " << buff.size() << std::endl;
    } else
        std::cout << "(sensors::publishSensors) Check srial connection" << std::endl;
}

bool initSerial(void) {
    /// Get the port in which our device is connected
    std::string deviceNode("2-3.3:1.0");
    std::string output = getStdoutFromCommand("ls -l /sys/class/tty/ttyUSB*");
    std::string port = "/dev" + output.substr(output.find(deviceNode) + deviceNode.size(), 8);

    std::cout << "(sensors::initSerial) STM32 port : " << port << std::endl;

    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
    serialConnection.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();

    if(serialConnection.isOpen())
        return true;
    else
        return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensors");

    ros::NodeHandle nh;

    bumper_pub = nh.advertise<gobot_base::BumperMsg>("bumpers_topic", 50);
    ir_pub = nh.advertise<gobot_base::IrMsg>("ir_topic", 50);
    proximity_pub = nh.advertise<gobot_base::ProximityMsg>("proximity_topic", 50);
    sonar_pub = nh.advertise<gobot_base::SonarMsg>("sonar_topic", 50);
    weight_pub = nh.advertise<gobot_base::WeightMsg>("weight_topic", 50);
    battery_pub = nh.advertise<gobot_base::BatteryMsg>("battery_topic", 50);
    cliff_pub = nh.advertise<gobot_base::CliffMsg>("cliff_topic", 50);

    if(initSerial()){
        ros::Rate r(20);
        while(ros::ok()){
            ros::spinOnce();

            publishSensors();

            r.sleep();
        }
    }

    return 0;
}