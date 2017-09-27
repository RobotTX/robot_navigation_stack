#include <gobot_base/sensors.hpp>

#define ERROR_THRESHOLD 5

ros::Publisher bumper_pub;
ros::Publisher ir_pub;
ros::Publisher proximity_pub;
ros::Publisher sonar_pub;
ros::Publisher weight_pub;
ros::Publisher battery_pub;
ros::Publisher cliff_pub;
serial::Serial serialConnection;

int last_charging_current = -1;
bool charging = false;
int error_count = 0;
std::vector<uint8_t> cmd;
std::mutex connectionMutex;
bool display_data = false;

bool displaySensorData(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res){
    ROS_INFO("(sensors::displaySensorData) Service called");
    display_data = (req.voltage > 0);
    return true;
}

void resetStm(void){
    if(serialConnection.isOpen()){
        connectionMutex.lock();
        serialConnection.write(std::vector<uint8_t>({0xD0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}));

        std::vector<uint8_t> buff;
        serialConnection.read(buff, 7);

        serialConnection.flush();

        connectionMutex.unlock();

        ROS_INFO("(sensors::publishSensors) resetStm");
    } else 
        ROS_ERROR("(sensors::publishSensors) Check serial connection 1");
}

bool isChargingService(gobot_msg_srv::IsCharging::Request &req, gobot_msg_srv::IsCharging::Response &res){
    res.isCharging = charging;

    return true;
}

/// get the output of the given system command
//tx//feedback string from executing command
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
        bool error = false;

        connectionMutex.lock();
        serialConnection.write(std::vector<uint8_t>({0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1}));

        std::vector<uint8_t> buff;
        serialConnection.read(buff, 47);

        serialConnection.flush();

        connectionMutex.unlock();

        //ROS_INFO("(sensors::publishSensors) Info : %lu %d %d %d", buff.size(), (int) buff.at(0), (int) buff.at(1), (int) buff.at(2));

        /// check if the 16 is useful
        if(buff.size() == 47){
            /// First 3 bytes are the sensor address, the command and the data length so we can ignore it
            /// Then comes sonars data
            gobot_msg_srv::SonarMsg sonar_data;
            sonar_data.distance1 = (buff.at(3) << 8) | buff.at(4);
            sonar_data.distance2 = (buff.at(5) << 8) | buff.at(6);
            sonar_data.distance3 = (buff.at(7) << 8) | buff.at(8);
            sonar_data.distance4 = (buff.at(9) << 8) | buff.at(10);
            sonar_data.distance5 = (buff.at(11) << 8) | buff.at(12);
            sonar_data.distance6 = (buff.at(13) << 8) | buff.at(14);
            sonar_data.distance7 = (buff.at(15) << 8) | buff.at(16);

            if(sonar_data.distance1 + sonar_data.distance2 + sonar_data.distance3 + sonar_data.distance4 
                + sonar_data.distance5 + sonar_data.distance6 + sonar_data.distance7 != 0)
                sonar_pub.publish(sonar_data);
            else {
                error = true;
                ROS_ERROR("(sensors::publishSensors) Check sonars data : %d %d %d %d %d %d %d", sonar_data.distance1, sonar_data.distance2, sonar_data.distance3,
                sonar_data.distance4, sonar_data.distance5, sonar_data.distance6, sonar_data.distance7);
            }

            /// Bumpers data
            gobot_msg_srv::BumperMsg bumper_data;
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
            gobot_msg_srv::IrMsg ir_data;
            ir_data.rearSignal = buff.at(18);
            ir_data.leftSignal = buff.at(19);
            ir_data.rightSignal = buff.at(20);
            ir_pub.publish(ir_data);

            /// Proximity sensors
            gobot_msg_srv::ProximityMsg proximity_data;
            proximity_data.signal1 = (buff.at(21) & 0b00000001) > 0;
            proximity_data.signal2 = (buff.at(21) & 0b00000010) > 0;
            proximity_pub.publish(proximity_data);

            /// Cliff sensors
            gobot_msg_srv::CliffMsg cliff_data;
            cliff_data.cliff1 = (buff.at(22) << 8) | buff.at(23);
            cliff_data.cliff2 = (buff.at(24) << 8) | buff.at(25);
            cliff_data.cliff3 = (buff.at(26) << 8) | buff.at(27);
            cliff_data.cliff4 = (buff.at(28) << 8) | buff.at(29);
            cliff_pub.publish(cliff_data);

            /// Battery data
            gobot_msg_srv::BatteryMsg battery_data;
            battery_data.BatteryStatus  = buff.at(31);
            battery_data.BatteryVoltage = (buff.at(32) << 8) | buff.at(33);
            battery_data.ChargingCurrent = (buff.at(34) << 8) | buff.at(35);
            battery_data.Temperature = (buff.at(36) << 8) | buff.at(37);
            battery_data.RemainCapacity = ((buff.at(38) << 8) | buff.at(39))/100;
            battery_data.FullCapacity = ((buff.at(40) << 8) | buff.at(41))/100;

            if(battery_data.BatteryVoltage == 0 || battery_data.ChargingCurrent == 0 
                || battery_data.FullCapacity == 0
                || battery_data.Temperature < 0){
                error = true;
                ROS_ERROR("(sensors::publishSensors) Check battery data : %d %d %d %d", (int) battery_data.BatteryVoltage, battery_data.ChargingCurrent,
                battery_data.FullCapacity, battery_data.Temperature);
            } else {
                if(battery_data.ChargingCurrent > 700 || (last_charging_current > 0 && battery_data.ChargingCurrent - last_charging_current > 60))
                    battery_data.ChargingFlag = true;
                else 
                    battery_data.ChargingFlag = false;

                charging = battery_data.ChargingFlag;
                last_charging_current = battery_data.ChargingCurrent;
                
                battery_pub.publish(battery_data);
            }


            /// Weight data
            gobot_msg_srv::WeightMsg weight_data;
            weight_data.weightInfo = (buff.at(43) << 8) | buff.at(44);
            weight_pub.publish(weight_data);

            /// External button
            int32_t external_button = buff.at(45);
            /// TODO do whatever we want with it

            if(display_data){
                std::cout << "Sonars : " << sonar_data.distance1 << " " << sonar_data.distance2 << " " << sonar_data.distance3 << " " << sonar_data.distance4 
                << " " << sonar_data.distance5 << " " << sonar_data.distance6 << " " << sonar_data.distance7 <<
                "\nBumpers : "  << (int) bumper_data.bumper1 << " " << (int) bumper_data.bumper2 << " " << (int) bumper_data.bumper3 << " " << (int) bumper_data.bumper4 << " "
                << (int) bumper_data.bumper5 << " " << (int) bumper_data.bumper6 << " " << (int) bumper_data.bumper7 << " " << (int) bumper_data.bumper8 <<
                "\nIr signals : " << (int) ir_data.rearSignal << " "  << (int) ir_data.leftSignal << " "  << (int) ir_data.rightSignal <<
                "\nProximity : " << (int) proximity_data.signal1 << " " << (int) proximity_data.signal2 <<
                "\nCliff : " << cliff_data.cliff1 << " " << cliff_data.cliff2 << " " << cliff_data.cliff3 << " " << cliff_data.cliff4 <<
                "\nBattery : " << battery_data.BatteryStatus << " " << battery_data.BatteryVoltage << " " << battery_data.ChargingCurrent << " " << battery_data.Temperature 
                << " " << battery_data.RemainCapacity << " " << battery_data.FullCapacity << " " << battery_data.ChargingFlag <<
                "\nWeight : " << weight_data.weightInfo <<
                "\nExternal button : " << external_button << std::endl;
            }


            /// The last byte is the Frame Check Sum and is not used

        } else {
            ROS_INFO("(sensors::publishSensors) Check buff size : %lu", buff.size());
            error = true;
        }

        if(error)
            error_count++;
        else
            error_count = 0;

        /// If we got more than <ERROR_THRESHOLD> errors in a row, we send a command to reset the stm32
        if(error_count > ERROR_THRESHOLD){
            resetStm();
            error_count = 0;
        }
            
    } else
        ROS_ERROR("(sensors::publishSensors) Check serial connection 2");
}

bool initSerial(void) {
    /// Get the port in which our device is connected
    std::string deviceNode("2-3.2:1.0");
    std::string output = getStdoutFromCommand("ls -l /sys/class/tty/ttyUSB*");
    std::string port = "/dev" + output.substr(output.find(deviceNode) + deviceNode.size(), 8);

    ROS_INFO("(sensors::initSerial) STM32 port : %s", port.c_str());

    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
    //14400 bytes per sec
    serialConnection.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();

    if(serialConnection.isOpen()){
        ROS_INFO("Connection established to STM32.");
        return true;
    }
    else
        return false;
}

bool setLedCallback(gobot_msg_srv::LedStrip::Request &req, gobot_msg_srv::LedStrip::Response &res){
    if(serialConnection.isOpen()){
        cmd={req.data[0],req.data[1],req.data[2],req.data[3],req.data[4],req.data[5],req.data[6],req.data[7],req.data[8],req.data[9],req.data[10]};

        connectionMutex.lock();
        serialConnection.write(cmd);
        std::vector<uint8_t> buff;
        serialConnection.read(buff, 47);
        serialConnection.flush();
        connectionMutex.unlock();
        return true;
    }
    else{
        return false;
    }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensors");

    ros::NodeHandle nh;

    bumper_pub = nh.advertise<gobot_msg_srv::BumperMsg>("bumpers_topic", 50);
    ir_pub = nh.advertise<gobot_msg_srv::IrMsg>("ir_topic", 50);
    proximity_pub = nh.advertise<gobot_msg_srv::ProximityMsg>("proximity_topic", 50);
    sonar_pub = nh.advertise<gobot_msg_srv::SonarMsg>("sonar_topic", 50);
    weight_pub = nh.advertise<gobot_msg_srv::WeightMsg>("weight_topic", 50);
    battery_pub = nh.advertise<gobot_msg_srv::BatteryMsg>("battery_topic", 50);
    cliff_pub = nh.advertise<gobot_msg_srv::CliffMsg>("cliff_topic", 50);

    ros::ServiceServer isChargingSrv = nh.advertiseService("isCharging", isChargingService);
    ros::ServiceServer setLedSrv = nh.advertiseService("/gobot_base/setLed", setLedCallback);

    ros::ServiceServer displayDataService = nh.advertiseService("displaySensorData", displaySensorData);

    if(initSerial()){
        ros::Rate r(5);
        while(ros::ok()){
            ros::spinOnce();
            publishSensors();

            r.sleep();
        }
    }

    return 0;
}