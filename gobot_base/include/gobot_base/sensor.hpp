//ros headers
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <numeric> 
#include <serial/serial.h>
#include <thread>

#include <gobot_msg_srv/set_robot_class.h>

#define ERROR_THRESHOLD 3

std::vector<uint8_t> REQUEST_DATA_CMD = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B};
std::vector<uint8_t> RESET_MCU_CMD =    {0xD0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B};
std::vector<uint8_t> SHUT_DOWN_CMD =    {0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x1B};

class SensorClass {
    public:
        SensorClass():
        battery_percent_(50), robot_status_(-1), charging_flag_(false), low_battery_(false), led_flag_(true),
        last_charging_current_(0), error_count_(0), mute_(0), charge_check_(0)
        {
            ros::NodeHandle nh;
            nh.getParam("SENSOR_BYTES", SENSOR_BYTES);  //61/49
            nh.getParam("SENSOR_PORT", SENSOR_PORT);
            nh.getParam("SENSOR_RATE", SENSOR_RATE);
            nh.getParam("USE_BUMPER", USE_BUMPER);
            nh.getParam("USE_SONAR", USE_SONAR);
            nh.getParam("USE_CLIFF", USE_CLIFF);
            nh.getParam("WEIGHT_MAX",max_weight_);
            nh.getParam("restart_file",restart_script);
            //0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
            led_color_ = {{"green",0x42}, {"blue",0x52}, {"yellow",0x59}, {"red",0x47}, {"cyan",0x43}, {"white",0x57}, {"magenta",0x4D}, {"off",0x00}};

            SetRobot_.initialize();

            initSerial();

            int fail=0, success=0;
            ros::Rate r2(2);
            //checking procedure
            ros::Time initilize_time = ros::Time::now();
            while(success<4 && ros::ok()){
                if((ros::Time::now()-initilize_time)>ros::Duration(10.0)){
                    //restart the system as unable to initilize MCU
                    std::string cmd;
                    cmd = "sudo sh " + restart_script;
                    system(cmd.c_str());
                    exit(1);
                }
                else{
                    if (checkSensors())
                        success++;
                    else
                        fail++;
                    
                    if(fail>5){
                        initSerial();
                        fail=0;
                        success=0;
                    }
                    ROS_INFO("(SENSORS::StartUp) SUCCESS:%d,  FAIL:%d, CC:%d", success,fail,last_charging_current_);
                }
                r2.sleep();
            }

            sensors_pub_ = nh.advertise<gobot_msg_srv::AllSensorsMsg>("/gobot_base/sensors_topic", 1);

            ir_pub_ = nh.advertise<gobot_msg_srv::IrMsg>("/gobot_base/ir_topic", 1);
            button_pub_ = nh.advertise<std_msgs::Int8>("/gobot_base/button_topic",1);
            gyro_pub_ = nh.advertise<gobot_msg_srv::GyroMsg>("/gobot_base/gyro_topic",1);
            sonar_pub_ = nh.advertise<gobot_msg_srv::SonarMsg>("/gobot_base/sonar_topic", 1);
            cliff_pub_ = nh.advertise<gobot_msg_srv::CliffMsg>("/gobot_base/cliff_topic", 1);
            weight_pub_ = nh.advertise<gobot_msg_srv::WeightMsg>("/gobot_base/weight_topic", 1);
            temperature_pub_ = nh.advertise<std_msgs::Float32>("/gobot_base/temperature_topic",1);
            battery_pub_ = nh.advertise<gobot_msg_srv::BatteryMsg>("/gobot_base/battery_topic", 1);
            bumper_pub_ = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_raw_topic", 1);
            proximity_pub_ = nh.advertise<gobot_msg_srv::ProximityMsg>("/gobot_base/proximity_topic", 1);

            useSonarSrv = nh.advertiseService("/gobot_base/use_sonar", &SensorClass::useSonarSrvCallback, this);
            useCliffSrv = nh.advertiseService("/gobot_base/use_cliff", &SensorClass::useCliffSrvCallback, this);
            useBumperSrv = nh.advertiseService("/gobot_base/use_bumper", &SensorClass::useBumperSrvCallback, this);
            shutdownSrv = nh.advertiseService("/gobot_base/shutdown_robot", &SensorClass::shutdownSrvCallback, this);
            setChargingSrv = nh.advertiseService("/gobot_base/set_charging", &SensorClass::setChargingSrvCallback, this);

            mute_sub_ = nh.subscribe("/gobot_status/mute", 1, &SensorClass::muteCallback, this);
            led_sub_ = nh.subscribe("/gobot_base/set_led", 1, &SensorClass::ledCallback, this);
            sound_sub_ = nh.subscribe("/gobot_base/set_sound", 1, &SensorClass::soundCallback, this);
            status_sub_ = nh.subscribe("/gobot_status/gobot_status", 1, &SensorClass::statusCallback, this);

            ledTimer_ = nh.createTimer(ros::Duration(30), &SensorClass::ledTimerCallback, this);
            
            sensor_thread_ = new boost::thread(&SensorClass::sensorThread, this);

            //Startup begin
            sensorsReadySrv = nh.advertiseService("/gobot_startup/sensors_ready", &SensorClass::sensorsReadySrvCallback,this);
            //Startup end
        }

        ~SensorClass(){
            led_sub_.shutdown();
            sound_sub_.shutdown();
            mute_sub_.shutdown();
            status_sub_.shutdown();
            ledTimer_.stop();

            sensor_thread_->interrupt();
            sensor_thread_->join();
            delete sensor_thread_;

            serialMutex_.lock();
            try{
                //Turn off LED when shut down node
                serialConnection.write(std::vector<uint8_t>({0xB0,0x03,0x01,0x57,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B}));
                std::vector<uint8_t> buff; 
                serialConnection.read(buff, 5);
                serialConnection.flush();
                serialConnection.close();
            } catch (std::exception& e) {
                ROS_ERROR("(SENSORS) Shutdown exception : %s", e.what());
            }
            serialMutex_.unlock();
        }

        bool initSerial(void){
            serialMutex_.lock(); 
            if(serialConnection.isOpen()){
                serialConnection.close();
            }
            ros::Duration(2.0).sleep();
            ROS_INFO("(SENSORS::initSerial) STM32 port : %s", SENSOR_PORT.c_str());
            // Set the serial port, baudrate and timeout in milliseconds
            serialConnection.setPort(SENSOR_PORT);
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
                ROS_INFO("(SENSORS::Initial) Reseted STM32. Received %lu bytes", buff.size());
                //we need wait for a while when startup robot and initilize STM
                ros::Duration(2.0).sleep();
                if(buff.size() != 5){
                    ROS_INFO("(SENSORS::Initial) Receive wrong ACK from STM32.");
                    serialMutex_.unlock(); 
                    return false;
                }
                ROS_INFO("(SENSORS::Initial) Receive correct ACK from STM32. Established connection to STM32.");
                serialMutex_.unlock(); 
                return true;
            }
            serialMutex_.unlock(); 

            return false;
        }

        bool checkSensors(){
            std::vector<uint8_t> buff = writeAndRead(REQUEST_DATA_CMD,SENSOR_BYTES);
            if(buff.size() == SENSOR_BYTES){
                if(buff.at(17)){
                    int16_t voltage = (buff.at(32) << 8) | buff.at(33);
                    int16_t temperature = (buff.at(36) << 8) | buff.at(37);
                    
                    if(voltage>0 && temperature>0){
                        last_charging_current_ = (buff.at(34) << 8) | buff.at(35);
                        return true;
                    }
                    else
                        ROS_ERROR("(SENSORS::CheckSensors) Check battery data : Voltage:%d, Temperature:%d", voltage,temperature);
                }
                else
                    ROS_ERROR("(SENSORS::CheckSensors) Bumpers information is wrong");
            }
            else
                ROS_ERROR("(SENSORS::CheckSensors) Wrong size : %zu", buff.size());

            return false;
        }

        void sensorThread(){
            last_led_time_ = ros::Time::now();
            reset_wifi_time_ = ros::Time::now();
            ros::Rate r(SENSOR_RATE);
            //start publish sensor information after checking procedure
            while(ros::ok()){
                std::vector<uint8_t> buff = writeAndRead(REQUEST_DATA_CMD,SENSOR_BYTES);

                bool error = false;
                bool error_bumper = false;
                bool error_weight = false;
                //ROS_INFO("(SENSORS::publishSensors) Info : %lu %d %d %d", buff.size(), (int) buff.at(0), (int) buff.at(1), (int) buff.at(2));
                /// check if the 16 is useful
                if(buff.size() == SENSOR_BYTES){
                    /// First 3 bytes are: 
                    // sensor address=B0, command=B1, data length=B2, so we can ignore it
                    //D for data, B for buff

                    gobot_msg_srv::AllSensorsMsg sensors_msg;
                    

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

                    sensors_msg.sonar = sonar_data;

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
            
                    sensors_msg.bumper = bumper_data;

                    /// Ir signals = D16-D18 / B18-B20
                    gobot_msg_srv::IrMsg ir_data;
                    ir_data.rearSignal = buff.at(18);
                    ir_data.leftSignal = buff.at(19);
                    ir_data.rightSignal = buff.at(20);
                    //ROS_INFO("(SENSORS::publishSensors) Check IR data : %d %d %d", ir_data.rearSignal,ir_data.leftSignal,ir_data.rightSignal);

                    sensors_msg.infrared = ir_data;

                    /// Proximity sensors = D19 / B21
                    gobot_msg_srv::ProximityMsg proximity_data;
                    proximity_data.signal1 = (buff.at(21) & 0b00000001) > 0;
                    proximity_data.signal2 = (buff.at(21) & 0b00000010) > 0;

                    sensors_msg.proximity = proximity_data;

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
                    //ROS_INFO("(SENSORS) %d,%d,%d,%d",cliff_data.cliff1,cliff_data.cliff2,cliff_data.cliff3,cliff_data.cliff4);
                    
                    sensors_msg.cliff = cliff_data;


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
                        ROS_ERROR("(SENSORS::publishSensors ERROR) Check battery data : Status:%d, Voltage:%d, ChargingCurrent:%d, Temperature:%d", 
                        battery_data.BatteryStatus,(int) battery_data.BatteryVoltage, battery_data.ChargingCurrent, battery_data.Temperature);
                    } 
                    else {
                        //battery data update slower than requesting rate
                        if(battery_data.ChargingCurrent!=last_charging_current_){
                            int current_diff = battery_data.ChargingCurrent - last_charging_current_;
                            battery_data.ChargingFlag = battery_data.ChargingCurrent<-500 ? false : true;
                            if(!charging_flag_){
                                charge_check_ = battery_data.ChargingFlag ? charge_check_+1 : 0;
                            }
                        }
                        else{
                            battery_data.ChargingFlag = charging_flag_;
                        }

                        if(charging_flag_!=battery_data.ChargingFlag){
                            if(battery_data.ChargingFlag && charge_check_>1){
                                charging_flag_ = true;
                                std::thread t1(&SensorClass::threadFunc, this, 1);
                                t1.detach();
                                //disable led change when charging
                                led_flag_ = false;
                                displayBatteryLed();
                            }
                            else{
                                if(!battery_data.ChargingFlag){
                                    std::thread t2(&SensorClass::threadFunc, this, 2);
                                    t2.detach();
                                    //enable led change when not charging
                                    if(!led_flag_){
                                        led_flag_ = true;
                                        displayBatteryLed();
                                    }
                                }
                                charging_flag_ = false;
                                battery_data.ChargingFlag = false;
                            }
                        }
                        battery_percent_ = battery_data.BatteryStatus;
                        last_charging_current_ = battery_data.ChargingCurrent;
                    }

                    sensors_msg.battery = battery_data;

                    /// Weight data = D40-D42 / B42-B44 
                    int32_t load_weight = (buff.at(42) << 16) | (buff.at(43) << 8) | buff.at(44);
                    if(load_weight > 2*max_weight_){
                        error_weight = true;
                        load_weight = 0;
                    }
                    else if(load_weight > max_weight_){
                        load_weight = -(load_weight - max_weight_);
                    }
                    gobot_msg_srv::WeightMsg weight_data;
                    //gram -> kg
                    weight_data.weight = load_weight/1000.0; 

                    sensors_msg.weight = weight_data;

                    /// External button 1-No press; 0-press = D43 / B45
                    int8_t external_button = buff.at(45);
                    std_msgs::Int8 button;
                    button.data = !external_button;

                    sensors_msg.button = button;

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
                    gyro_pub_.publish(gyro);

                    sensors_msg.gyro = gyro;

                    /// Temperature = D58-D59 / B60-B61
                    temperature.data = (((buff.at(60)<<8)|buff.at(61))-21)/333.87 + 21.0;
                    temperature_pub_.publish(temperature);

                    sensors_msg.temperature = temperature;

                    /// ACK = B62
                    sensors_pub_.publish(sensors_msg);
                    
                    /// The last byte is the Frame Check Sum and is not used
                    if(!error){
                        //If no error, publish sensor data
                        ir_pub_.publish(ir_data);
                        proximity_pub_.publish(proximity_data);
                        button_pub_.publish(button);
                        battery_pub_.publish(battery_data);

                        if (!error_weight)
                            weight_pub_.publish(weight_data);
                        
                        if(USE_BUMPER && !error_bumper)
                            bumper_pub_.publish(bumper_data);

                        if (USE_SONAR)
                            sonar_pub_.publish(sonar_data);

                        if (USE_CLIFF)
                            cliff_pub_.publish(cliff_data);
                    
                        if(resetwifi_button==1 && (ros::Time::now()-reset_wifi_time_)>ros::Duration(1.0)){
                            reset_wifi_time_ = ros::Time::now();
                            setSound(1,2);
                            //if scanning, save the map
                            if(robot_status_<=25 && robot_status_>=20){ 
                                ROS_INFO("(SENSORS) Save scanned map");   
                                std::thread t3(&SensorClass::threadFunc, this, 3);
                                t3.detach();
                                
                            }
                            //otherwise, reset the wifi
                            else{
                                ROS_INFO("(SENSORS) Reset robot WiFi"); 
                                std::thread t4(&SensorClass::threadFunc, this, 4);
                                t4.detach();
                            }
                        }
                    }
                    else if (error_bumper)
                        ROS_ERROR("(SENSORS::publishSensors ERROR) All bumpers got a collision");
                } 
                else {
                    //ROS_ERROR("(SENSORS::publishSensors) Wrong buff size : %lu, error count: %d", buff.size(),error_count_);
                    error = true;
                }

                error_count_ = error ? error_count_+1 : 0;

                if(error_count_ > ERROR_THRESHOLD){
                /// If we got more than <ERROR_THRESHOLD> errors in a row, we send a command to reset the stm32
                    //SetRobot_.setMotorSpeed('F', 0, 'F', 0);
                    initSerial();
                    error_count_ = 0;
                }
                r.sleep();
            }
        }

        void threadFunc(int n){
            if(n==1){
                SetRobot_.setDock(1);
                if(robot_status_ == 5)
                    SetRobot_.setStatus(-1,"ROBOT_READY");
            }
            else if(n==2){
                SetRobot_.setDock(0);
            }
            else if(n==3){
                std_srvs::Empty empty_srv;                
                ros::service::call("/gobot_scan/save_map",empty_srv);
            }
            else if(n==4){
                SetRobot_.setWifi("","");
            }
        }

        bool setChargingSrvCallback(gobot_msg_srv::SetBool::Request &req, gobot_msg_srv::SetBool::Response &res){
            charging_flag_ = req.data;
            return true;
        }

        bool useBumperSrvCallback(gobot_msg_srv::SetBool::Request &req, gobot_msg_srv::SetBool::Response &res){
            USE_BUMPER = req.data;
            return true;
        }

        bool useSonarSrvCallback(gobot_msg_srv::SetBool::Request &req, gobot_msg_srv::SetBool::Response &res){
            USE_SONAR = req.data;
            return true;
        }

        bool useCliffSrvCallback(gobot_msg_srv::SetBool::Request &req, gobot_msg_srv::SetBool::Response &res){
            USE_CLIFF = req.data;
            return true;
        }

        bool shutdownSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
            //ROS_INFO("(SENSORS) Receive a LED change request, and succeed to execute.");
            SetRobot_.setMotorSpeed('F', 0, 'F', 0);
            setSound(2,1);
            //shutdown command
            std::vector<uint8_t> buff = writeAndRead(SHUT_DOWN_CMD,5);
            while(buff.size()!=5){
                ROS_INFO("(SENSORS::Shutdown) Shutdown system. Received %lu bytes", buff.size());
                buff = writeAndRead(SHUT_DOWN_CMD,5);
                ros::Duration(1.0).sleep();
            }
            return true;
        }

        bool sensorsReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
            return true;
        }

        void soundCallback(const gobot_msg_srv::SoundMsg::ConstPtr& sound){
            setSound(sound->num,sound->time_on);
        }

        void ledCallback(const gobot_msg_srv::LedMsg::ConstPtr& led){
            if(led->mode == -1){
                displayBatteryLed();
                setSound(1,2);
            }
            //change led if not low battery and able to change led
            else if(!low_battery_ && led_flag_){
                setLed(led->mode,led->color);
            }
        }

        void muteCallback(const std_msgs::Int8::ConstPtr& msg){
            mute_ = msg->data;
        }

        void statusCallback(const std_msgs::Int8::ConstPtr& msg){
            robot_status_ = msg->data;
            //enable led change when moving away from CS
            if(!led_flag_ && (robot_status_==5 || robot_status_==25)){
                setLed(1,{"green","white"});
                led_flag_ = true;
            }
        }

        void ledTimerCallback(const ros::TimerEvent&){
            //for every 30 sec, we check robot status in order to change led
            low_battery_ = battery_percent_<10 ? true : false;
            //change led if able to change led
            if(led_flag_){
                if (low_battery_){
                    setLed(0,{"magenta"});
                }
                //Show battery status if no stage for certain period, show battery status
                else if ((ros::Time::now()-last_led_time_)>ros::Duration(300.0)){
                    displayBatteryLed();
                }   
            }
            else{
                displayBatteryLed();
            }

            //if docking or exploring, perdically sound twice
            if(robot_status_==15 || robot_status_==25)
                setSound(2,1);
            //change led if low battery and able to change led
            else if(low_battery_ && led_flag_)
                setSound(3,2);
        }


    private:
        std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0){
            std::vector<uint8_t> buff;
            
            serialMutex_.lock();
            if(serialConnection.isOpen()){
                try{
                    /// Send bytes to the MD49
                    size_t bytes_wrote = serialConnection.write(toWrite);
                    /// Read any byte that we are expecting
                    if(bytesToRead > 0)
                        serialConnection.read(buff, bytesToRead);

                    //serialConnection.flush();
                } catch (std::exception& e) {
                    ROS_ERROR("(SENSORS) exception : %s", e.what());
                }
            }
            else {
                ROS_WARN("(SENSORS::writeAndRead) The serial connection is not opened, something is wrong");
            }
            serialMutex_.unlock();

            return buff;
        }

        std::string getStdoutFromCommand(std::string cmd){
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

        void displayBatteryLed(void){
            std::string color;
            if(battery_percent_<25)
                color = "magenta";
            else if(battery_percent_<75)
                color = "yellow";
            else if(battery_percent_<=100)
                color = "cyan";
            
            if(led_flag_ || battery_percent_==100)
                setLed(0,{color});
            else
                setLed(1,{color,"white"});
        }

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
            last_led_time_ = ros::Time::now();
            //ROS_INFO("(SENSORS) Receive a LED change request, and succeed to execute.");
            return true;
        }

        bool setSound(int num,int time_on){
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
            //ROS_INFO("(SENSORS) Receive a sound request, and succeed to execute.");
            return true;
        }


        std::string SENSOR_PORT, restart_script;

        int16_t last_charging_current_;
        bool charging_flag_, low_battery_, led_flag_, USE_BUMPER, USE_SONAR, USE_CLIFF;
        int battery_percent_, error_count_, mute_, max_weight_, robot_status_, charge_check_;
        int SENSOR_RATE, SENSOR_BYTES;

        serial::Serial serialConnection;
        std::mutex serialMutex_;
        std::map<std::string, uint8_t> led_color_;
 
        ros::Time last_led_time_, reset_wifi_time_;
        ros::Timer ledTimer_;
        ros::Publisher sensors_pub_, bumper_pub_, ir_pub_, proximity_pub_, sonar_pub_, weight_pub_, battery_pub_, cliff_pub_, button_pub_, gyro_pub_, temperature_pub_;
        ros::ServiceServer shutdownSrv, displayDataSrv, sensorsReadySrv, setChargingSrv, useBumperSrv, useSonarSrv, useCliffSrv;
        ros::Subscriber led_sub_, sound_sub_, mute_sub_, status_sub_;

        boost::thread *sensor_thread_;

        robot_class::SetRobot SetRobot_;
};

