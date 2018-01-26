#include <gobot_base/sensors.hpp>

#define ERROR_THRESHOLD 2

#define BATTERY_MAX 23000
#define BATTERY_MIN 21500

ros::Publisher bumper_pub, ir_pub, proximity_pub, sonar_pub, weight_pub, battery_pub, cliff_pub, button_pub;
serial::Serial serialConnection;

std_srvs::Empty empty_srv;
std::string user_name;
std::vector<uint8_t> cmd;
std::mutex connectionMutex;
std::string STMdevice;
gobot_msg_srv::GetGobotStatus get_gobot_status;

ros::Time last_led_time;

int last_charging_current = 0;
bool charging = false, low_battery = false;
int battery_percent = 50;

int error_count = 0;
bool display_data = false;
int STM_CHECK = 0, STM_RATE=5, charge_check = 0;;
bool USE_BUMPER=true,USE_SONAR=true,USE_CLIFF=true;

robot_class::SetRobot robot;

std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead){
    std::vector<uint8_t> buff;
    connectionMutex.lock();
    try{
        /// Send bytes to the MD49
        size_t bytes_wrote = serialConnection.write(toWrite);
        /// Read any byte that we are expecting
        if(bytesToRead > 0)
            serialConnection.read(buff, bytesToRead);

        //serialConnection.flush();
    } catch (std::exception& e) {
        ROS_ERROR("(Sensors) exception : %s", e.what());
    }
    /// Unlock the mutex
    connectionMutex.unlock();

    return buff;
}

bool resetSTMSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    resetStm();

    return true;
}

bool displaySensorData(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res){
    ROS_INFO("(sensors::displaySensorData) Service called");
    display_data = (req.voltage > 0);
    return true;
}

void resetStm(void){
    if(serialConnection.isOpen()){
        std::vector<uint8_t> buff;
        std::vector<uint8_t> toWrite = {0xD0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B};
        connectionMutex.lock();
        try{
            /// Send bytes to the MD49
            size_t bytes_wrote = serialConnection.write(toWrite);
            /// Read any byte that we are expecting
            serialConnection.read(buff, 5);
        } catch (std::exception& e) {
            ROS_ERROR("(Sensors) exception : %s", e.what());
        }
        /// Unlock the mutex
        ros::Duration(1.0).sleep();
        connectionMutex.unlock();
        ROS_WARN("(sensors::publishSensors WARN) Reseted STM32. Received %lu bytes", buff.size());
    } 
    else 
        ROS_ERROR("(sensors::publishSensors ERROR) Check serial connection 1");
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


/*     
       B5 B6 B7 B8     
          IR_B
       C3       C4
       S3 P2 P1 S4
        ---BB---
        |      |
  IR_R  |      | IR_L
        |__FF__|
        S2    S1
        C2    C1
       B4 B3 B2 B1
*/

void publishSensors(void){
    if(serialConnection.isOpen()){
        bool error = false;
        bool error_bumper = false;
        std::vector<uint8_t> buff = writeAndRead({0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B},49);

        //ROS_INFO("(sensors::publishSensors) Info : %lu %d %d %d", buff.size(), (int) buff.at(0), (int) buff.at(1), (int) buff.at(2));
        /// check if the 16 is useful
        if(buff.size() == 49){
            /// First 3 bytes are the sensor address, the command and the data length, so we can ignore it
            //7 sonars, 8 bumpers, 3 IR, 2 distance, 4 cliff, 1 battery, 1 load


            /// Sonars data = D3-D16
            //1->front_right,2->front_left,3->back_left,4->back_right
            gobot_msg_srv::SonarMsg sonar_data;
            sonar_data.distance1 = (buff.at(3) << 8) | buff.at(4);
            sonar_data.distance2 = (buff.at(5) << 8) | buff.at(6);
            sonar_data.distance3 = (buff.at(7) << 8) | buff.at(8);
            sonar_data.distance4 = (buff.at(9) << 8) | buff.at(10);
            sonar_data.distance5 = (buff.at(11) << 8) | buff.at(12);
            sonar_data.distance6 = (buff.at(13) << 8) | buff.at(14);
            sonar_data.distance7 = (buff.at(15) << 8) | buff.at(16);


            /// Bumpers data = D17
            gobot_msg_srv::BumperMsg bumper_data;

            if(buff.at(17)){
                /*for old gobot
                bumper_data.bumper1 = (buff.at(17) & 0b00000001) > 0;
                bumper_data.bumper2 = (buff.at(17) & 0b00000010) > 0;
                bumper_data.bumper3 = (buff.at(17) & 0b00000100) > 0;
                bumper_data.bumper4 = (buff.at(17) & 0b00001000) > 0;
                bumper_data.bumper5 = (buff.at(17) & 0b00010000) > 0;
                bumper_data.bumper6 = (buff.at(17) & 0b00100000) > 0;
                bumper_data.bumper7 = (buff.at(17) & 0b01000000) > 0;
                bumper_data.bumper8 = (buff.at(17) & 0b10000000) > 0;
                */
                bumper_data.bumper6 = (buff.at(17) & 0b00000001) > 0;
                bumper_data.bumper3 = (buff.at(17) & 0b00000010) > 0;
                bumper_data.bumper4 = (buff.at(17) & 0b00000100) > 0;
                bumper_data.bumper1 = (buff.at(17) & 0b00001000) > 0;
                bumper_data.bumper5 = (buff.at(17) & 0b00010000) > 0;
                bumper_data.bumper2 = (buff.at(17) & 0b00100000) > 0;
                bumper_data.bumper7 = (buff.at(17) & 0b01000000) > 0;
                bumper_data.bumper8 = (buff.at(17) & 0b10000000) > 0;
            } 
            else{
                error_bumper = true;
            }


            /// Ir signals = D18 ~ D20
            gobot_msg_srv::IrMsg ir_data;
            ir_data.rearSignal = buff.at(18);
            ir_data.leftSignal = buff.at(19);
            ir_data.rightSignal = buff.at(20);
            //ROS_INFO("(sensors::publishSensors) Check IR data : %d %d %d", ir_data.rearSignal,ir_data.leftSignal,ir_data.rightSignal);


            /// Proximity sensors = D21
            gobot_msg_srv::ProximityMsg proximity_data;
            proximity_data.signal1 = (buff.at(21) & 0b00000001) > 0;
            proximity_data.signal2 = (buff.at(21) & 0b00000010) > 0;


            /// Cliff sensors = D22 ~ D29
            gobot_msg_srv::CliffMsg cliff_data;
            cliff_data.cliff1 = (buff.at(22) << 8) | buff.at(23);
            cliff_data.cliff2 = (buff.at(24) << 8) | buff.at(25);
            cliff_data.cliff3 = (buff.at(26) << 8) | buff.at(27);
            cliff_data.cliff4 = (buff.at(28) << 8) | buff.at(29);
            cliff_data.cliff1 = (cliff_data.cliff1>1000) ? 1 : cliff_data.cliff1;
            cliff_data.cliff2 = (cliff_data.cliff2>1000) ? 1 : cliff_data.cliff2;
            cliff_data.cliff3 = (cliff_data.cliff3>1000) ? 1 : cliff_data.cliff3;
            cliff_data.cliff4 = (cliff_data.cliff4>1000) ? 1 : cliff_data.cliff4;
            //ROS_INFO("%d,%d,%d,%d",cliff_data.cliff1,cliff_data.cliff2,cliff_data.cliff3,cliff_data.cliff4);


            /// Battery data
            gobot_msg_srv::BatteryMsg battery_data;
            battery_data.BatteryStatus  = buff.at(31);
            battery_data.BatteryVoltage = (buff.at(32) << 8) | buff.at(33);
            battery_data.ChargingCurrent = (buff.at(34) << 8) | buff.at(35);
            battery_data.Temperature = (buff.at(36) << 8) | buff.at(37);
            battery_data.RemainCapacity = ((buff.at(38) << 8) | buff.at(39))/100;
            battery_data.FullCapacity = ((buff.at(40) << 8) | buff.at(41))/100;

            //double percentage = battery_data.BatteryVoltage;
            //percentage=(percentage-BATTERY_MIN)/(BATTERY_MAX-BATTERY_MIN); 
            double percentage = battery_data.BatteryStatus/100.0;
            if(percentage>1.0)
                battery_data.Percentage=1.0;
            else if(percentage<0.0)
                battery_data.Percentage=0.0;
            else
                battery_data.Percentage=percentage;

            if(battery_data.BatteryVoltage == 0 || battery_data.Temperature <= 0){
                error = true;
                ROS_ERROR("(sensors::publishSensors ERROR) Check battery data : Status:%d, Voltage:%d, ChargingCurrent:%d, Temperature:%d", 
                battery_data.BatteryStatus,(int) battery_data.BatteryVoltage, battery_data.ChargingCurrent, battery_data.Temperature);
            } 
            else {
                //battery data update slower than requesting rate
                if(abs(battery_data.ChargingCurrent-last_charging_current) > 5){
                    int current_diff = battery_data.ChargingCurrent - last_charging_current;
                    if(battery_data.BatteryStatus<100){
                        if (battery_data.ChargingCurrent < 0){
                            if(current_diff > 100)
                                battery_data.ChargingFlag = true;
                            else
                                battery_data.ChargingFlag = false;
                        }
                        else if (battery_data.ChargingCurrent >1500){
                            if(current_diff < -100)
                                battery_data.ChargingFlag = false;
                            else
                                battery_data.ChargingFlag = true;
                        }
                        else{
                            if(current_diff > 50)
                                battery_data.ChargingFlag = true;
                            else
                                battery_data.ChargingFlag = false;
                        }
                    }
                    else{
                        if (battery_data.ChargingCurrent > 0){
                            if(current_diff < -50)
                                battery_data.ChargingFlag = false;
                            else
                                battery_data.ChargingFlag = true;
                        }
                        else{
                            if(current_diff > 100)
                                battery_data.ChargingFlag = true;
                            else
                                battery_data.ChargingFlag = false;
                        }
                    }
                    if(!charging){
                        charge_check = battery_data.ChargingFlag ? charge_check+1 : 0;
                    }
                }
                else{
                    battery_data.ChargingFlag = charging;
                }

                if(charging != battery_data.ChargingFlag){
                    if(battery_data.ChargingFlag && charge_check>1){
                        charging = true;
                        robot.setDock(1);
                        displayBatteryLed();
                    }
                    else{
                        charging = false;
                        if(!battery_data.ChargingFlag){
                            robot.setDock(0);
                            displayBatteryLed();
                        }
                        battery_data.ChargingFlag = false;
                    }
                }
                battery_percent = battery_data.BatteryStatus;
                last_charging_current = battery_data.ChargingCurrent;
            }


            /// Weight data
            gobot_msg_srv::WeightMsg weight_data;
            weight_data.weightInfo = (buff.at(43) << 8) | buff.at(44);


            /// External button 1-No press; 0-press
            int32_t external_button = buff.at(45);
            std_msgs::Int8 button;
            button.data = !external_button;

            int32_t engage_point = buff.at(46);

            /// Reset wifi button (double click power button)
            int32_t resetwifi_button = buff.at(47);


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
                "\nWeight : " << weight_data.weightInfo << " " <<
                "\nExternal button : " << external_button << " " <<
                "\nResetwifi button : " << resetwifi_button << " " << 
                "\nengage point : " << engage_point << " " << 
                std::endl;
            }

            /// The last byte is the Frame Check Sum and is not used
            if(!error && STM_CHECK>=10){
                //If no error, publish sensor data
                ir_pub.publish(ir_data);
                proximity_pub.publish(proximity_data);
                weight_pub.publish(weight_data);
                button_pub.publish(button);
                battery_pub.publish(battery_data);

                if (USE_SONAR)
                    sonar_pub.publish(sonar_data);

                if (USE_CLIFF && !battery_data.ChargingFlag)
                    cliff_pub.publish(cliff_data);
                    
                if(USE_BUMPER && !error_bumper)
                    bumper_pub.publish(bumper_data);
                else if(error_bumper)
                    ROS_ERROR("(sensors::publishSensors ERROR) All bumpers got a collision");
            
                if(resetwifi_button==1){
                    ros::service::call("/gobot_status/get_gobot_status",get_gobot_status);
                    //if scanning, save the map
                    if(get_gobot_status.response.status==20 || get_gobot_status.response.status==25 || get_gobot_status.response.status==21){                        
                        ros::service::call("/gobot_scan/save_map",empty_srv);
                        
                    }
                    //otherwise, reset the wifi
                    else{
                        robot.setWifi("","");
                    }
                }
            }
        } 
        else {
            //ROS_ERROR("(sensors::publishSensors) Wrong buff size : %lu, error count: %d", buff.size(),error_count);
            error = true;
        }

        error_count = error ? error_count+1 : 0;

        if(error_count > ERROR_THRESHOLD){
        /// If we got more than <ERROR_THRESHOLD> errors in a row, we send a command to reset the stm32
            //robot.setMotorSpeed('F', 0, 'F', 0);
            resetStm();
            error_count = 0;
        }

        if(!error){
            STM_CHECK++;
        }
    } 
    else{
        //Try reopen STM32 serial
        initSerial();
        ROS_ERROR("(sensors::publishSensors error) Check serial connection 2");
    }
}

bool setSoundSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    return setSound(req.data[0],req.data[1]);
}

bool setSound(int num,int time_on, int time_off){
    gobot_msg_srv::GetInt get_mute;
    ros::service::call("/gobot_status/get_mute",get_mute);
    //mute.
    if(get_mute.response.data[0])
        return true;

    //serial open and not mute
    if(serialConnection.isOpen()){
        uint8_t n = num;
        uint8_t time_off = (n==1) ? 0x00 : 0x96;
        std::vector<uint8_t> sound_cmd;
        switch (time_on){
            //100ms
            case 1:
                sound_cmd={0xE0, 0x01, 0x05, n, 0x00, 0x64, 0X00, time_off, 0X00, 0X00, 0x1B};
                break;
            //400ms
            case 2:
                sound_cmd={0xE0, 0x01, 0x05, n, 0x01, 0x90, 0X00, time_off, 0X00, 0X00, 0x1B};
                break;
            //800ms
            default:
                sound_cmd={0xE0, 0x01, 0x05, n, 0x03, 0x20, 0X00, time_off, 0X00, 0X00, 0x1B};
                break;
        }
        std::vector<uint8_t> buff = writeAndRead(sound_cmd,5);
        ROS_INFO("Receive a sound request, and succeed to execute.");
        return true;
    }
    else{
        ROS_WARN("Receive a sound request, but failed to execute.");
        return false;
    } 
}

bool showBatteryLedsrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    displayBatteryLed();
    setSound(1,2);
    return true;
}

bool setLedSrvCallback(gobot_msg_srv::LedStrip::Request &req, gobot_msg_srv::LedStrip::Response &res){
    if(!low_battery || charging){
        //ROS_INFO("Receive a LED change request, and succeed to execute.");
        return setLed({req.data[0],req.data[1],req.data[2],req.data[3],req.data[4],req.data[5],req.data[6],req.data[7],req.data[8],req.data[9],req.data[10]});
    }
}

bool setLed(std::vector<uint8_t> cmd){
    if(serialConnection.isOpen()){
        std::vector<uint8_t> led_cmd=cmd;
        std::vector<uint8_t> buff = writeAndRead(led_cmd,5);
        last_led_time = ros::Time::now();
        ROS_INFO("Receive a LED change request, and succeed to execute.");
        return true;
    }
    else{
        ROS_INFO("Receive a LED change request, but failed to execute.");
        return false;
    } 
}

void displayBatteryLed(){
    std::vector<uint8_t> led_cmd;
    std::vector<uint8_t> led_buff;
    if(!charging){
        if(battery_percent<25)
            led_cmd={0xB0,0x03,0x01,0x4D,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B};
        else if(battery_percent<75)
            led_cmd={0xB0,0x03,0x01,0x59,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B};
        else if(battery_percent<100)
            led_cmd={0xB0,0x03,0x01,0x43,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B};
    }
    else{
        if(battery_percent<25)
            led_cmd={0xB0,0x01,0x02,0x4D,0x57,0x00,0x00,0x00,0x00,0x96,0x1B};
        else if(battery_percent<75)
            led_cmd={0xB0,0x01,0x02,0x59,0x57,0x00,0x00,0x00,0x00,0x96,0x1B};
        else if(battery_percent<100)
            led_cmd={0xB0,0x01,0x02,0x43,0x57,0x00,0x00,0x00,0x00,0x96,0x1B};
        else
            led_cmd={0xB0,0x03,0x01,0x43,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B};
    }
    setLed(led_cmd);
}

void ledTimerCallback(const ros::TimerEvent&){
    //0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
    if (!charging){
        if (battery_percent < 10){
            setLed({0xB0,0x03,0x01,0x4D,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B});
            ros::service::call("/gobot_status/get_gobot_status",get_gobot_status);
            if(get_gobot_status.response.status!=15){
                setSound(3,2);
            }
            low_battery = true;
        }
        //Show battery status if no stage for certain period, show battery lvl
        else {
            low_battery = false;
            if (((ros::Time::now() - last_led_time).toSec()>300.0)){
                displayBatteryLed();
            }
        }   
    }
}


bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(serialConnection.isOpen()){
        //ROS_INFO("Receive a LED change request, and succeed to execute.");
        robot.setMotorSpeed('F', 0, 'F', 0);
        //shutdown command
        std::vector<uint8_t> buff = writeAndRead({0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1B},5);

        //turn led white
        buff.clear();
        buff = writeAndRead({0xB0,0x03,0x01,0x57,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B},5);
        return true;
    }
    else{
        ROS_WARN("Receive a shutdown request, but failed to execute.");
        return false;
    } 

}


bool sensorsReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return true;
}

bool initSerial(void) {
    std::string port = STMdevice;
    
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
        resetStm();
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
            writeAndRead({0xB0,0x03,0x01,0x57,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B},5);
        }
    } catch (std::exception& e) {
		ROS_ERROR("(Sensors) Shutdown exception : %s", e.what());
	}

    std::string joy_cmd = "rosnode kill /joy_node &";
    system(joy_cmd.c_str());

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensors");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);


    //Startup begin
    ROS_INFO("(Sensors) Waiting for MD49 to be ready...");
    ros::service::waitForService("/gobot_startup/motor_ready", ros::Duration(60.0));
    ROS_INFO("(Sensors) MD49 is ready.");
    //Startup end

    nh.getParam("STMdevice", STMdevice);
    nh.getParam("STM_RATE", STM_RATE);
    nh.getParam("USE_BUMPER", USE_BUMPER);
    nh.getParam("USE_SONAR", USE_SONAR);
    nh.getParam("USE_CLIFF", USE_CLIFF);
    nh.getParam("user_name", user_name);

    ros::Timer ledTimer = nh.createTimer(ros::Duration(60), ledTimerCallback);
    last_led_time = ros::Time::now();

    bumper_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_raw_topic", 50);
    ir_pub = nh.advertise<gobot_msg_srv::IrMsg>("/gobot_base/ir_topic", 50);
    proximity_pub = nh.advertise<gobot_msg_srv::ProximityMsg>("/gobot_base/proximity_topic", 50);
    sonar_pub = nh.advertise<gobot_msg_srv::SonarMsg>("/gobot_base/sonar_topic", 50);
    weight_pub = nh.advertise<gobot_msg_srv::WeightMsg>("/gobot_base/weight_topic", 50);
    battery_pub = nh.advertise<gobot_msg_srv::BatteryMsg>("/gobot_base/battery_topic", 50);
    cliff_pub = nh.advertise<gobot_msg_srv::CliffMsg>("/gobot_base/cliff_topic", 50);
    button_pub = nh.advertise<std_msgs::Int8>("/gobot_base/button_topic",50);
    ros::ServiceServer shutdownSrv = nh.advertiseService("/gobot_base/shutdown_robot", shutdownSrvCallback);
    ros::ServiceServer setLedSrv = nh.advertiseService("/gobot_base/setLed", setLedSrvCallback);
    ros::ServiceServer setSoundSrv = nh.advertiseService("/gobot_base/setSound", setSoundSrvCallback);
    ros::ServiceServer displayDataSrv = nh.advertiseService("/gobot_base/displaySensorData", displaySensorData);
    ros::ServiceServer resetSTMSrv = nh.advertiseService("/gobot_base/reset_STM", resetSTMSrvCallback);
    ros::ServiceServer showBatteryLed = nh.advertiseService("/gobot_base/show_Battery_LED", showBatteryLedsrvCallback);

    ros::ServiceServer sensorsReadySrv;

    if(initSerial()){
        ros::Rate r(STM_RATE);
        while(ros::ok()){
            ros::spinOnce();
            publishSensors();
            r.sleep();

            if(STM_CHECK==10){
                //Startup begin
                std::string joy_cmd = "rosrun joy joy_node &";
                system(joy_cmd.c_str());
                sensorsReadySrv = nh.advertiseService("/gobot_startup/sensors_ready", sensorsReadySrvCallback);
                //Startup end
            }
        }
    }

    return 0;
}