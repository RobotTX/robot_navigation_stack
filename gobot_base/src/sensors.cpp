#include <gobot_base/sensors.hpp>

#define ERROR_THRESHOLD 3


int16_t last_charging_current = 0;
bool charging = false, low_battery = false;
int battery_percent = 50;

int error_count = 0;
bool display_data = false;
int STM_CHECK = 0, STM_RATE=5, charge_check = 0, STM_BYTES = 0;
bool USE_BUMPER=true,USE_SONAR=true,USE_CLIFF=true;

int mute_=0;

int max_weight = 200000;

ros::Publisher bumper_pub, ir_pub, proximity_pub, sonar_pub, weight_pub, battery_pub, cliff_pub, button_pub, gyro_pub, temperature_pub;
ros::ServiceServer setLedSrv, setSoundSrv;
ros::Time last_led_time, reset_wifi_time;
serial::Serial serialConnection;

std_srvs::Empty empty_srv;
std::vector<uint8_t> cmd;
std::mutex connectionMutex;
std::string STMdevice;
std::map<std::string, uint8_t> led_color_;

std::string restart_file;

robot_class::SetRobot SetRobot;
robot_class::GetRobot GetRobot;
int robot_status_ = -1;

std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead){
    std::vector<uint8_t> buff;
    
    connectionMutex.lock();
    if(serialConnection.isOpen()){
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
    }
    else {
        ROS_WARN("(wheels::writeAndRead) The serial connection is not opened, something is wrong");
    }
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
    connectionMutex.lock(); 
    serialConnection.write(RESET_MCU_CMD);
    /// Read any byte that we are expecting
    std::vector<uint8_t> buff;
    serialConnection.read(buff, 5);
    /// Unlock the mutex
    ros::Duration(0.5).sleep();
    connectionMutex.unlock();
    ROS_WARN("(sensors::publishSensors WARN) Reseted STM32. Received %lu bytes", buff.size());
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

bool checkSensors(){
    std::vector<uint8_t> buff = writeAndRead(REQUEST_DATA_CMD,STM_BYTES);
    if(buff.size() == STM_BYTES){
        if(buff.at(17)){
            int16_t voltage = (buff.at(32) << 8) | buff.at(33);
            int16_t temperature = (buff.at(36) << 8) | buff.at(37);
            
            if(voltage>0 && temperature>0){
                last_charging_current = (buff.at(34) << 8) | buff.at(35);
                return true;
            }
            else
                ROS_ERROR("(sensors::CheckSensors) Check battery data : Voltage:%d, Temperature:%d", voltage,temperature);
        }
        else
            ROS_ERROR("(sensors::CheckSensors) Bumpers information is wrong");
    }
    else
        ROS_ERROR("(sensors::CheckSensors) Wrong size : %zu", buff.size());

    return false;
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
    bool error = false;
    bool error_bumper = false;
    bool error_weight = false;
    std::vector<uint8_t> buff = writeAndRead(REQUEST_DATA_CMD,STM_BYTES);

    //ROS_INFO("(sensors::publishSensors) Info : %lu %d %d %d", buff.size(), (int) buff.at(0), (int) buff.at(1), (int) buff.at(2));
    /// check if the 16 is useful
    if(buff.size() == STM_BYTES){
        /// First 3 bytes are: 
        // sensor address=B0, command=B1, data length=B2, so we can ignore it
        //D for data, B for buff
        /// Sonars data = D1-D14 / B3-B16
        //1->front_right,2->front_left,3->back_left,4->back_right
        gobot_msg_srv::SonarMsg sonar_data;
        sonar_data.distance1 = (buff.at(3) << 8) | buff.at(4);
        sonar_data.distance2 = (buff.at(5) << 8) | buff.at(6);
        sonar_data.distance3 = (buff.at(7) << 8) | buff.at(8);
        sonar_data.distance4 = (buff.at(9) << 8) | buff.at(10);
        /*
        sonar_data.distance5 = (buff.at(11) << 8) | buff.at(12);
        sonar_data.distance6 = (buff.at(13) << 8) | buff.at(14);
        sonar_data.distance7 = (buff.at(15) << 8) | buff.at(16);
        */
        sonar_data.distance5 = -1;
        sonar_data.distance6 = -1;
        sonar_data.distance7 = -1;


        /// Bumpers data = D15 / B17
        gobot_msg_srv::BumperMsg bumper_data;

        if(buff.at(17)){
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
            error = true;
        }


        /// Ir signals = D16-D18 / B18-B20
        gobot_msg_srv::IrMsg ir_data;
        ir_data.rearSignal = buff.at(18);
        ir_data.leftSignal = buff.at(19);
        ir_data.rightSignal = buff.at(20);
        //ROS_INFO("(sensors::publishSensors) Check IR data : %d %d %d", ir_data.rearSignal,ir_data.leftSignal,ir_data.rightSignal);


        /// Proximity sensors = D19 / B21
        gobot_msg_srv::ProximityMsg proximity_data;
        proximity_data.signal1 = (buff.at(21) & 0b00000001) > 0;
        proximity_data.signal2 = (buff.at(21) & 0b00000010) > 0;


        /// Cliff sensors = D20-D27 / B22-B29
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
        
        //D28 / B30  MAX ERROR

        /// Battery data = D29-D39 / B31-B41
        gobot_msg_srv::BatteryMsg battery_data;
        battery_data.BatteryStatus  = buff.at(31);
        battery_data.BatteryVoltage = (buff.at(32) << 8) | buff.at(33);
        battery_data.ChargingCurrent = (buff.at(34) << 8) | buff.at(35);
        battery_data.Temperature = (buff.at(36) << 8) | buff.at(37);
        battery_data.RemainCapacity = ((buff.at(38) << 8) | buff.at(39))/100;
        battery_data.FullCapacity = ((buff.at(40) << 8) | buff.at(41))/100;
        battery_data.Percentage = battery_data.BatteryStatus/100.0;

        if(battery_data.BatteryVoltage <= 0 || battery_data.Temperature <= 0){
            error = true;
            ROS_ERROR("(sensors::publishSensors ERROR) Check battery data : Status:%d, Voltage:%d, ChargingCurrent:%d, Temperature:%d", 
            battery_data.BatteryStatus,(int) battery_data.BatteryVoltage, battery_data.ChargingCurrent, battery_data.Temperature);
        } 
        else {
            //battery data update slower than requesting rate
            if(abs(battery_data.ChargingCurrent-last_charging_current) > 5){
                int current_diff = battery_data.ChargingCurrent - last_charging_current;
                //if not fully charged
                if(battery_data.BatteryStatus<93){
                    if(battery_data.ChargingCurrent >2500){
                        battery_data.ChargingFlag = true;
                    }
                    else if(battery_data.ChargingCurrent < -1000){
                        battery_data.ChargingFlag = false;
                    }
                    else if(battery_data.ChargingCurrent >1000){
                        battery_data.ChargingFlag = current_diff<-100 ? false: true;
                    }
                    else if(battery_data.ChargingCurrent >=0){
                        battery_data.ChargingFlag = current_diff>100 ? true : false;
                    }
                    else{
                        battery_data.ChargingFlag = current_diff>200 ? true : false;
                    }
                }
                //if fully charged
                else{
                    battery_data.ChargingFlag = battery_data.ChargingCurrent<-250 ? false : true;
                }
                if(!charging){
                    charge_check = battery_data.ChargingFlag ? charge_check+1 : 0;
                }
            }
            else{
                battery_data.ChargingFlag = charging;
            }

            if(charging!=battery_data.ChargingFlag){
                if(battery_data.ChargingFlag && charge_check>1){
                    charging = true;
                    std::thread([](){
                        SetRobot.setDock(1);
                    }).detach();
                    displayBatteryLed();
                }
                else{
                    charging = false;
                    if(!battery_data.ChargingFlag){
                        std::thread([](){
                            SetRobot.setDock(0);
                        }).detach();
                        displayBatteryLed();
                    }
                    battery_data.ChargingFlag = false;
                }
            }
            battery_percent = battery_data.BatteryStatus;
            last_charging_current = battery_data.ChargingCurrent;
        }


        /// Weight data = D40-D42 / B42-B44 
        int32_t load_weight = (buff.at(42) << 16) | (buff.at(43) << 8) | buff.at(44);
        if(load_weight > 2*max_weight){
            error_weight = true;
            load_weight = 0;
        }
        else{
            load_weight = load_weight > max_weight ? -(load_weight-max_weight): load_weight;
        }
        gobot_msg_srv::WeightMsg weight_data;
        //gram -> kg
        weight_data.weight = load_weight/1000.0; 


        /// External button 1-No press; 0-press = D43 / B45
        int8_t external_button = buff.at(45);
        std_msgs::Int8 button;
        button.data = !external_button;

        /// Engage point (not in use) = D44 / B46
        int8_t engage_point = buff.at(46);

        /// Reset wifi button (double click power button) = D45 / B47
        int8_t resetwifi_button = buff.at(47);

        /// Gyro+Accelerametor = D46-D57 / B48-B59
        gobot_msg_srv::GyroMsg gyro;
        std_msgs::Float32 temperature;
        //if 1st bit is 1, means negative
        gyro.accelx =  buff.at(48)<<8 | buff.at(49);
        gyro.accely =  buff.at(50)<<8 | buff.at(51); 
        gyro.accelz =  buff.at(52)<<8 | buff.at(53);
        gyro.gyrox = buff.at(54)<<8 | buff.at(55);
        gyro.gyroy = buff.at(56)<<8 | buff.at(57); 
        gyro.gyroz = buff.at(58)<<8 | buff.at(59);
        gyro_pub.publish(gyro);

        /// Temperature = D58-D59 / B60-B61
        temperature.data = (((buff.at(60)<<8)|buff.at(61))-21)/333.87 + 21.0;
        temperature_pub.publish(temperature);

        /// ACK = B62

        if(display_data){
            std::cout << "Sonars : " << sonar_data.distance1 << " " << sonar_data.distance2 << " " << sonar_data.distance3 << " " << sonar_data.distance4 
            << " " << sonar_data.distance5 << " " << sonar_data.distance6 << " " << sonar_data.distance7 <<
            "\nBumpers : "  << bumper_data.bumper1 << " " << bumper_data.bumper2 << " " << bumper_data.bumper3 << " " << bumper_data.bumper4 << " "
            << bumper_data.bumper5 << " " << bumper_data.bumper6 << " " << bumper_data.bumper7 << " " << bumper_data.bumper8 <<
            "\nIr signals : " << ir_data.rearSignal << " "  << ir_data.leftSignal << " "  << ir_data.rightSignal <<
            "\nProximity : " << proximity_data.signal1 << " " << proximity_data.signal2 <<
            "\nCliff : " << cliff_data.cliff1 << " " << cliff_data.cliff2 << " " << cliff_data.cliff3 << " " << cliff_data.cliff4 <<
            "\nBattery : " << battery_data.BatteryStatus << " " << battery_data.BatteryVoltage << " " << battery_data.ChargingCurrent << " " << battery_data.Temperature 
            << " " << battery_data.RemainCapacity << " " << battery_data.FullCapacity << " " << battery_data.ChargingFlag <<
            "\nWeight : " << weight_data.weight << " " <<
            "\nExternal button : " << external_button << " " <<
            "\nResetwifi button : " << resetwifi_button << " " << 
            "\nengage point : " << engage_point << " " <<
            "\ngyro data : " << gyro.gyrox <<" "<< gyro.gyroy <<" "<< gyro.gyroz <<" "<< gyro.accelx <<" "<< gyro.accely <<" "<< gyro.accelz <<
            "\ntemperature : "<< temperature.data <<
            std::endl;
        }

        /// The last byte is the Frame Check Sum and is not used
        if(!error){
            //If no error, publish sensor data
            ir_pub.publish(ir_data);
            proximity_pub.publish(proximity_data);
            button_pub.publish(button);
            battery_pub.publish(battery_data);

            if (!error_weight)
                weight_pub.publish(weight_data);
            
            if(USE_BUMPER && !error_bumper)
                bumper_pub.publish(bumper_data);

            if (USE_SONAR)
                sonar_pub.publish(sonar_data);

            if (USE_CLIFF && !battery_data.ChargingFlag)
                cliff_pub.publish(cliff_data);
        
            if(resetwifi_button==1 && (ros::Time::now()-reset_wifi_time).toSec()>1.0){
                reset_wifi_time = ros::Time::now();
                setSound(1,2);
                //if scanning, save the map
                if(robot_status_<=25 && robot_status_>=20){ 
                    ROS_INFO("Save scanned map");     
                    std::thread([](){                  
                        ros::service::call("/gobot_scan/save_map",empty_srv);
                    }).detach();
                    
                }
                //otherwise, reset the wifi
                else{
                    ROS_INFO("Reset robot WiFi"); 
                    std::thread([](){
                        SetRobot.setWifi("","");
                    }).detach();
                }
            }
        }
        else if (error_bumper)
            ROS_ERROR("(sensors::publishSensors ERROR) All bumpers got a collision");
    } 
    else {
        //ROS_ERROR("(sensors::publishSensors) Wrong buff size : %lu, error count: %d", buff.size(),error_count);
        error = true;
    }

    error_count = error ? error_count+1 : 0;

    if(error_count > ERROR_THRESHOLD){
    /// If we got more than <ERROR_THRESHOLD> errors in a row, we send a command to reset the stm32
        //SetRobot.setMotorSpeed('F', 0, 'F', 0);
        initSerial();
        error_count = 0;
    }
}

void ledCallback(const gobot_msg_srv::LedMsg::ConstPtr& led){
    if(led->mode == -1){
        displayBatteryLed();
        setSound(1,2);
    }
    else if(!low_battery && !charging){
        setLed(led->mode,led->color);
    }
}


void soundCallback(const gobot_msg_srv::SoundMsg::ConstPtr& sound){
    setSound(sound->num,sound->time_on);
}

void muteCallback(const std_msgs::Int8::ConstPtr& msg){
    mute_ = msg->data;
}

void statusCallback(const std_msgs::Int8::ConstPtr& msg){
    robot_status_ = msg->data;
}

bool setSoundSrvCallback(gobot_msg_srv::SetIntArray::Request &req, gobot_msg_srv::SetIntArray::Response &res){
    return setSound(req.data[0],req.data[1]);
}

bool setLedSrvCallback(gobot_msg_srv::LedStrip::Request &req, gobot_msg_srv::LedStrip::Response &res){
    //if low battery and not charging, only show magenta color to warn user
    //if charging, but robot received move command, color also changed
    if(!low_battery && !charging){
        //ROS_INFO("Receive a LED change request, and succeed to execute.");
        return setLed(req.mode,req.color);
    }
    return false;
}

bool setSound(int num, int time_on){
    if(mute_)
        return true;

    uint8_t n = num;
    uint8_t time_off = n==1 ? 0x00 : 0x96;
    std::vector<uint8_t> sound_cmd = {0xE0, 0x01, 0x05, n, 0x00, 0x64, 0X00, time_off, 0X00, 0X00, 0x1B};
    switch (time_on){
        //100ms
        case 1:
            sound_cmd[5] = 0x64;
            break;
        //400ms
        case 2:
            sound_cmd[4] = 0x01;
            sound_cmd[5] = 0x90;
            break;
        //800ms
        default:
            sound_cmd[4] = 0x03;
            sound_cmd[5] = 0x20;
            break;
    }
    std::vector<uint8_t> buff = writeAndRead(sound_cmd,5);
    //ROS_INFO("Receive a sound request, and succeed to execute.");
    return true;
}

void displayBatteryLed(){
    std::string color;
    if(battery_percent<25)
        color = "magenta";
    else if(battery_percent<75)
        color = "yellow";
    else if(battery_percent<=100)
        color = "cyan";

    if(!charging || battery_percent==100)
        setLed(0,{color});
    else
        setLed(1,{color,"white"});
}

//mode 0=permanent, mode 1=running
bool setLed(int mode, const std::vector<std::string> &color){
    //0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
    std::vector<uint8_t> led_cmd;
    switch (mode) {
        case 0:
            led_cmd = {0xB0,0x03,0x01,0x57,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B};
        break;

        case 1:
            led_cmd = {0xB0,0x01,0x02,0x42,0x57,0x00,0x00,0x00,0x00,0x96,0x1B};
        break;

        default:
            led_cmd = {0xB0,0x03,0x01,0x57,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B}; 
        break;
    }

    led_cmd[2]=color.size();
    for(int i=0;i<color.size();i++){
        led_cmd[3+i]=led_color_.at(color[i]);
    }
    std::vector<uint8_t> buff = writeAndRead(led_cmd,5);
    last_led_time = ros::Time::now();
    //ROS_INFO("Receive a LED change request, and succeed to execute.");
    return true;
}

void ledTimerCallback(const ros::TimerEvent&){
    //for every 30 sec, we change led
    low_battery = battery_percent<10 ? true : false;
    if (!charging){
        if (low_battery){
            setLed(0,{"magenta"});
        }
        //Show battery status if no stage for certain period, show battery lvl
        else if (((ros::Time::now() - last_led_time).toSec()>300.0)){
            displayBatteryLed();
        }   
    }
    else{
        displayBatteryLed();
    }

    //if docking or exploring, perdically sound twice
    if(robot_status_==15 || robot_status_==25)
        setSound(2,1);
    else if(low_battery && !charging)
        setSound(3,2);
}


bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //ROS_INFO("Receive a LED change request, and succeed to execute.");
    SetRobot.setMotorSpeed('F', 0, 'F', 0);
    setSound(2,1);
    //shutdown command
    std::vector<uint8_t> buff = writeAndRead(SHUT_DOWN_CMD,5);
    while(buff.size()!=5){
        ROS_INFO("(sensors::Shutdown) Shutdown system. Received %lu bytes", buff.size());
        buff = writeAndRead(SHUT_DOWN_CMD,5);
        ros::Duration(1.0).sleep();
    }
    return true;
}


bool sensorsReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return true;
}

void initData(ros::NodeHandle &nh){
    nh.getParam("STM_BYTES", STM_BYTES);  //61/49
    nh.getParam("STMdevice", STMdevice);
    nh.getParam("STM_RATE", STM_RATE);
    nh.getParam("USE_BUMPER", USE_BUMPER);
    nh.getParam("USE_SONAR", USE_SONAR);
    nh.getParam("USE_CLIFF", USE_CLIFF);
    nh.getParam("restart_file",restart_file);
    nh.getParam("max_weight",max_weight);
    
    //0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
    led_color_ = {{"green",0x42}, {"blue",0x52}, {"yellow",0x59}, {"red",0x47}, {"cyan",0x43}, {"white",0x57}, {"magenta",0x4D}, {"off",0x00}};
}

bool initSerial(){
    connectionMutex.lock(); 
    std::string port = STMdevice;
    if(serialConnection.isOpen()){
        serialConnection.close();
    }
    ros::Duration(2.0).sleep();
    ROS_INFO("(sensors::initSerial) STM32 port : %s", port.c_str());
    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
    //14400 bytes per sec
    serialConnection.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();

    if(serialConnection.isOpen()){
        //reset STM
        serialConnection.write(RESET_MCU_CMD);
        /// Read any byte that we are expecting
        std::vector<uint8_t> buff;
        serialConnection.read(buff, 5);
        ROS_INFO("(sensors::Initial) Reseted STM32. Received %lu bytes", buff.size());
        //we need wait for a while when startup robot and initilize STM
        ros::Duration(2.0).sleep();
        if(buff.size() != 5){
            ROS_INFO("(Sensors::Initial) Receive wrong ACK from STM32.");
            connectionMutex.unlock(); 
            return false;
        }
        ROS_INFO("(Sensors::Initial) Receive correct ACK from STM32. Established connection to STM32.");
        connectionMutex.unlock(); 
        return true;
    }
    connectionMutex.unlock(); 

    return false;
}

void mySigintHandler(int sig){   
    setLedSrv.shutdown();
    setSoundSrv.shutdown();

    connectionMutex.lock();
    try{
        //Turn off LED when shut down node
        serialConnection.write(std::vector<uint8_t>({0xB0,0x03,0x01,0x57,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B}));
        std::vector<uint8_t> buff; 
        serialConnection.read(buff, 5);
        serialConnection.close();
    } catch (std::exception& e) {
		ROS_ERROR("(Sensors) Shutdown exception : %s", e.what());
	}
    connectionMutex.unlock();

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensors");
    ros::NodeHandle nh;
    
    SetRobot.initialize();
    signal(SIGINT, mySigintHandler);

    //Startup begin
    ROS_INFO("(Sensors) Waiting for MD49 to be ready...");
    ros::service::waitForService("/gobot_startup/motor_ready", ros::Duration(60.0));
    ROS_INFO("(Sensors) MD49 is ready.");
    //Startup end

    initData(nh);

    bumper_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_raw_topic", 50);
    ir_pub = nh.advertise<gobot_msg_srv::IrMsg>("/gobot_base/ir_topic", 50);
    proximity_pub = nh.advertise<gobot_msg_srv::ProximityMsg>("/gobot_base/proximity_topic", 50);
    sonar_pub = nh.advertise<gobot_msg_srv::SonarMsg>("/gobot_base/sonar_topic", 50);
    weight_pub = nh.advertise<gobot_msg_srv::WeightMsg>("/gobot_base/weight_topic", 50);
    battery_pub = nh.advertise<gobot_msg_srv::BatteryMsg>("/gobot_base/battery_topic", 50);
    cliff_pub = nh.advertise<gobot_msg_srv::CliffMsg>("/gobot_base/cliff_topic", 50);
    button_pub = nh.advertise<std_msgs::Int8>("/gobot_base/button_topic",50);
    gyro_pub = nh.advertise<gobot_msg_srv::GyroMsg>("/gobot_base/gyro_topic",50);
    temperature_pub = nh.advertise<std_msgs::Float32>("/gobot_base/temperature_topic",50);
    
    initSerial();

    int n=0;
    ros::Rate r2(2);
    //checking procedure
    while(STM_CHECK<4 && ros::ok()){
        if (checkSensors()){
            STM_CHECK++;
            ROS_INFO("(Sensors::StartUp) Check Iteration:%d, CC:%d", STM_CHECK,last_charging_current);
        }
        else{
            n++;
        }
        if(n>10){
            initSerial();
            n=0;
        }
        r2.sleep();
    }

    //Declare service server after checking MCU
    ////Replace these two speed services with publisher & subscriber
    setLedSrv = nh.advertiseService("/gobot_base/setLed", setLedSrvCallback);
    setSoundSrv = nh.advertiseService("/gobot_base/setSound", setSoundSrvCallback);
    ////
    
    ros::Subscriber ledSubscriber = nh.subscribe("/gobot_base/set_led", 1, ledCallback);
    ros::Subscriber soundSubscriber = nh.subscribe("/gobot_base/set_sound", 1, soundCallback);
    ros::Subscriber muteSubscriber = nh.subscribe("/gobot_status/mute", 1, muteCallback);
    ros::Subscriber statusSubscriber = nh.subscribe("/gobot_status/gobot_status", 1, statusCallback);

    ros::ServiceServer shutdownSrv = nh.advertiseService("/gobot_base/shutdown_robot", shutdownSrvCallback);
    ros::ServiceServer resetSTMSrv = nh.advertiseService("/gobot_base/reset_STM", resetSTMSrvCallback);
    ros::ServiceServer displayDataSrv = nh.advertiseService("/gobot_base/displaySensorData", displaySensorData);
    ros::Timer ledTimer = nh.createTimer(ros::Duration(30), ledTimerCallback);
    last_led_time = ros::Time::now();
    reset_wifi_time = ros::Time::now();
    
    //Startup begin
    ros::ServiceServer sensorsReadySrv = nh.advertiseService("/gobot_startup/sensors_ready", sensorsReadySrvCallback);
    //Startup end

    ros::Rate r(STM_RATE);
    //start publish sensor information after checking procedure
    while(ros::ok()){
        publishSensors();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}