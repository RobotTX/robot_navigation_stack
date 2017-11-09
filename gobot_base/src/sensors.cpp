#include <gobot_base/sensors.hpp>

#define ERROR_THRESHOLD 1

#define BATTERY_MAX 23000
#define BATTERY_MIN 21500

ros::Publisher bumper_pub;
ros::Publisher ir_pub;
ros::Publisher proximity_pub;
ros::Publisher sonar_pub;
ros::Publisher weight_pub;
ros::Publisher battery_pub;
ros::Publisher cliff_pub;
ros::Publisher button_pub;
serial::Serial serialConnection;

int last_charging_current = -1;
bool charging = false;
int error_count = 0;
std::vector<uint8_t> cmd;
std::mutex connectionMutex;
bool display_data = false;
std::string STMdevice;
gobot_msg_srv::GetGobotStatus get_gobot_status;
ros::ServiceClient getGobotStatusSrv;

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    //ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR , velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("/gobot_motor/setSpeeds", speed);
}

bool resetMotorSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(serialConnection.isOpen()){
        connectionMutex.lock();
        serialConnection.write(std::vector<uint8_t>({0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}));
        std::vector<uint8_t> buff;
        serialConnection.read(buff, 40);
        serialConnection.flush();
        connectionMutex.unlock();
        ROS_WARN("(sensors::publishSensors) Reseted MD49 Motor Driver. Received %lu bytes", buff.size());
    } else 
        ROS_ERROR("(sensors::publishSensors) Check serial connection 1");
}

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
        serialConnection.read(buff, 40);
        serialConnection.flush();
        connectionMutex.unlock();

        ROS_WARN("(sensors::publishSensors WARN) Reseted STM32. Received %lu bytes", buff.size());

    } 
    else 
        ROS_ERROR("(sensors::publishSensors ERROR) Check serial connection 1");
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
        bool error_bumper = false;
        connectionMutex.lock();
        serialConnection.write(std::vector<uint8_t>({0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B}));

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

            /*
            if(sonar_data.distance1 == 0 && (sonar_data.distance2 == 0 || sonar_data.distance3 == 0)){
                error = true;
                ROS_ERROR("(sensors::publishSensors) Check sonars data : %d %d %d %d %d %d %d", sonar_data.distance1, sonar_data.distance2, sonar_data.distance3,
                sonar_data.distance4, sonar_data.distance5, sonar_data.distance6, sonar_data.distance7);
            }
            */

            /// Bumpers data
            gobot_msg_srv::BumperMsg bumper_data;

            if(buff.at(17)){
                bumper_data.bumper1 = (buff.at(17) & 0b00000001) > 0;
                bumper_data.bumper2 = (buff.at(17) & 0b00000010) > 0;
                bumper_data.bumper3 = (buff.at(17) & 0b00000100) > 0;
                bumper_data.bumper4 = (buff.at(17) & 0b00001000) > 0;
                bumper_data.bumper5 = (buff.at(17) & 0b00010000) > 0;
                bumper_data.bumper6 = (buff.at(17) & 0b00100000) > 0;
                bumper_data.bumper7 = (buff.at(17) & 0b01000000) > 0;
                bumper_data.bumper8 = (buff.at(17) & 0b10000000) > 0;
            } 
            else{
                ROS_ERROR("(sensors::publishSensors ERROR) All bumpers got a collision");
                error_bumper = true;
            }


            /// Ir signals
            gobot_msg_srv::IrMsg ir_data;
            ir_data.rearSignal = buff.at(18);
            ir_data.leftSignal = buff.at(19);
            ir_data.rightSignal = buff.at(20);
            //ROS_INFO("(sensors::publishSensors) Check IR data : %d %d %d", ir_data.rearSignal,ir_data.leftSignal,ir_data.rightSignal);

            /// Proximity sensors
            gobot_msg_srv::ProximityMsg proximity_data;
            proximity_data.signal1 = (buff.at(21) & 0b00000001) > 0;
            proximity_data.signal2 = (buff.at(21) & 0b00000010) > 0;

            /// Cliff sensors
            gobot_msg_srv::CliffMsg cliff_data;
            cliff_data.cliff1 = (buff.at(22) << 8) | buff.at(23);
            cliff_data.cliff2 = (buff.at(24) << 8) | buff.at(25);
            cliff_data.cliff3 = (buff.at(26) << 8) | buff.at(27);
            cliff_data.cliff4 = (buff.at(28) << 8) | buff.at(29);

            /// Battery data
            gobot_msg_srv::BatteryMsg battery_data;
            battery_data.BatteryStatus  = buff.at(31);
            battery_data.BatteryVoltage = (buff.at(32) << 8) | buff.at(33);
            battery_data.ChargingCurrent = (buff.at(34) << 8) | buff.at(35);
            battery_data.Temperature = (buff.at(36) << 8) | buff.at(37);
            battery_data.RemainCapacity = ((buff.at(38) << 8) | buff.at(39))/100;
            battery_data.FullCapacity = ((buff.at(40) << 8) | buff.at(41))/100;

            double percentage = battery_data.BatteryVoltage;
            percentage=(percentage-BATTERY_MIN)/(BATTERY_MAX-BATTERY_MIN); 
            if(percentage>1.0)
                battery_data.Percentage=1.0;
            else if(percentage<0.0)
                battery_data.Percentage=0.0;
            else
                battery_data.Percentage=percentage;

            if(battery_data.BatteryVoltage == 0 || battery_data.Temperature < 0){
                error = true;
                ROS_ERROR("(sensors::publishSensors ERROR) Check battery data : %d %d %d %d", (int) battery_data.BatteryVoltage, battery_data.ChargingCurrent,
                battery_data.FullCapacity, battery_data.Temperature);
            } 
            else {
                if(battery_data.ChargingCurrent != last_charging_current){
                    if(battery_data.ChargingCurrent < 2000 && (battery_data.ChargingCurrent - last_charging_current < -60))
                        battery_data.ChargingFlag = false;
                    else if(battery_data.ChargingCurrent > 1500 || (battery_data.ChargingCurrent > 1000 && (battery_data.ChargingCurrent - last_charging_current > 60)))
                        battery_data.ChargingFlag = true;
                    else
                        battery_data.ChargingFlag = false;
                } 
                else {
                    battery_data.ChargingFlag = charging;
                }

                last_charging_current = battery_data.ChargingCurrent;

                if(charging != battery_data.ChargingFlag){
                    charging = battery_data.ChargingFlag;
                    gobot_msg_srv::SetDockStatus set_dock_status;
                    if(charging){
                        set_dock_status.request.status = 1;
                        //~ROS_INFO("Gobot is charging");
                    }
                    else{
                        set_dock_status.request.status = 0;
                        //~ROS_INFO("Gobot is not charging");
                    }
                    ros::service::call("/gobot_status/set_dock_status",set_dock_status);
                }
            }


            /// Weight data
            gobot_msg_srv::WeightMsg weight_data;
            weight_data.weightInfo = (buff.at(43) << 8) | buff.at(44);

            /// External button 1-No press; 0-press
            int32_t external_button = buff.at(45);
            /// TODO do whatever we want with it
            //ROS_INFO("External button:%d",external_button);
            std_msgs::Int8 button;
            button.data=external_button;
            button_pub.publish(button);


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
            if(!error){
            sonar_pub.publish(sonar_data);
            ir_pub.publish(ir_data);
            proximity_pub.publish(proximity_data);
            cliff_pub.publish(cliff_data);
            battery_pub.publish(battery_data);
            weight_pub.publish(weight_data);
                if(!error_bumper)
                    bumper_pub.publish(bumper_data);
            }
        } 
        else {
            //ROS_INFO("(sensors::publishSensors) Check buff size : %lu", buff.size());
            error = true;
        }

        if(!error){
            error_count = 0;
        }
        else{
            error_count++;
        }

        /// If we got more than <ERROR_THRESHOLD> errors in a row, we send a command to reset the stm32
        if(error_count > ERROR_THRESHOLD){
            setSpeed('F', 0, 'F', 0);
            resetStm();
            //error may be caused by touching the power supply 
            getGobotStatusSrv.call(get_gobot_status);
            //go to docking state
            if(get_gobot_status.response.status==15){
                gobot_msg_srv::BatteryMsg battery_data;
                battery_data.ChargingFlag = true;
                battery_data.Temperature=-1;
                charging = battery_data.ChargingFlag;
                battery_pub.publish(battery_data);
                ROS_WARN("(sensors::publishSensors WARN) Error caused by charging battery at the moment.");
            }
            error_count = 0;
        }
            
    } 
    else{
        //Try reopen STM32 serial
        initSerial();
        ROS_ERROR("(sensors::publishSensors error) Check serial connection 2");
    }
}


bool setLedSrvCallback(gobot_msg_srv::LedStrip::Request &req, gobot_msg_srv::LedStrip::Response &res){
    if(serialConnection.isOpen()){
        ROS_INFO("Receive a LED change request, and succeed to execute.");

        std::vector<uint8_t> led_cmd={req.data[0],req.data[1],req.data[2],req.data[3],req.data[4],req.data[5],req.data[6],req.data[7],req.data[8],req.data[9],req.data[10]};

        connectionMutex.lock();
        serialConnection.write(led_cmd);
        std::vector<uint8_t> buff;
        serialConnection.read(buff,5);
        serialConnection.flush();
        connectionMutex.unlock();
        return true;
    }
    else{
        ROS_WARN("Receive a LED change request, but failed to execute.");
        return false;
    } 
}

bool setSoundSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    gobot_msg_srv::GetInt get_mute;
    ros::service::call("/gobot_status/get_mute",get_mute);
    //mute
    if (get_mute.response.data[0])
        return true;

    //serial open and not mute
    if(serialConnection.isOpen()){
        ROS_INFO("Receive a sound requestm, and succeed to execute.");
        uint8_t n = req.data[0];
        uint8_t on_time = 0x01;
        uint8_t off_time = 0x01;
        on_time = req.data[1];
        
        if(req.data.size()==3 && req.data[2]!=0)
            off_time = req.data[2];
            
        std::vector<uint8_t> sound_cmd({0xE0, 0x01, 0x05, n, on_time, 0x00, off_time, 0x00, 0x00, 0x00, 0x1B});

        connectionMutex.lock();
        serialConnection.write(sound_cmd);
        std::vector<uint8_t> buff;
        serialConnection.read(buff,5);
        serialConnection.flush();
        connectionMutex.unlock();
        return true;
    }
    else{
        ROS_WARN("Receive a sound request, but failed to execute.");
        return false;
    } 
}

bool initSerial(void) {
    /// Get the port in which our device is connected
    std::string deviceNode(STMdevice);
    std::string output = getStdoutFromCommand("ls -l /sys/class/tty/ttyUSB*");
    std::string port = "/dev" + output.substr(output.find(deviceNode) + deviceNode.size(), 8);

    ROS_INFO("(sensors::initSerial) STM32 port : %s", port.c_str());

    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
    //14400 bytes per sec
    serialConnection.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();

    if(serialConnection.isOpen()){
        
        ROS_INFO("Established connection to STM32.");
        return true;
    }
    else
        return false;
}

void mySigintHandler(int sig)
{   
    try{
        if(serialConnection.isOpen()){
            //Turn off LED when shut down node
            //0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
            connectionMutex.lock();
            serialConnection.write(std::vector<uint8_t>({0xB0,0x03,0x01,0x57,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B}));
            std::vector<uint8_t> buff;
            serialConnection.read(buff,5);
            serialConnection.flush();
            serialConnection.close();
            connectionMutex.unlock();
        }
    } catch (std::exception& e) {
		ROS_ERROR("(Sensors) exception : %s", e.what());
	}

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensors");

    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    getGobotStatusSrv = nh.serviceClient<gobot_msg_srv::GetGobotStatus>("/gobot_status/get_gobot_status");

    //Startup begin
    ROS_INFO("(Sensors) Waiting for MD49 to be ready...");
    getGobotStatusSrv.waitForExistence(ros::Duration(30.0));
    getGobotStatusSrv.call(get_gobot_status);
    while((get_gobot_status.response.status!=-1 || get_gobot_status.response.text!="MD49_READY") && ros::ok()){
        getGobotStatusSrv.call(get_gobot_status);
        ros::Duration(0.2).sleep();
    }
    ROS_INFO("(Sensors) MD49 is ready.");
    //Startup end

    nh.getParam("STMdevice", STMdevice);

    if(initSerial()){
        bumper_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_topic", 50);
        ir_pub = nh.advertise<gobot_msg_srv::IrMsg>("/gobot_base/ir_topic", 50);
        proximity_pub = nh.advertise<gobot_msg_srv::ProximityMsg>("/gobot_base/proximity_topic", 50);
        sonar_pub = nh.advertise<gobot_msg_srv::SonarMsg>("/gobot_base/sonar_topic", 50);
        weight_pub = nh.advertise<gobot_msg_srv::WeightMsg>("/gobot_base/weight_topic", 50);
        battery_pub = nh.advertise<gobot_msg_srv::BatteryMsg>("/gobot_base/battery_topic", 50);
        cliff_pub = nh.advertise<gobot_msg_srv::CliffMsg>("/gobot_base/cliff_topic", 50);
        button_pub = nh.advertise<std_msgs::Int8>("/gobot_base/button_topic",50);

        ros::ServiceServer isChargingSrv = nh.advertiseService("/gobot_status/charging_status", isChargingService);
        
        ros::ServiceServer setLedSrv = nh.advertiseService("/gobot_base/setLed", setLedSrvCallback);
        ros::ServiceServer setSoundSrv = nh.advertiseService("/gobot_base/setSound", setSoundSrvCallback);
        ros::ServiceServer displayDataSrv = nh.advertiseService("/gobot_base/displaySensorData", displaySensorData);
        ros::ServiceServer resetMotorSrv = nh.advertiseService("resetMotorDriver", resetMotorSrvCallback);

        //Startup begin
        gobot_msg_srv::SetGobotStatus set_gobot_status;
        set_gobot_status.request.status = -1;
        set_gobot_status.request.text ="STM32_READY";
        ros::service::call("/gobot_status/set_gobot_status",set_gobot_status);
        //Startup end

        ros::Rate r(5);
        while(ros::ok()){
            ros::spinOnce();
            publishSensors();

            r.sleep();
        }
    }

    return 0;
}