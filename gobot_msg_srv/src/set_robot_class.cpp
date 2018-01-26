#include <gobot_msg_srv/set_robot_class.h>


namespace robot_class {
    SetRobot::SetRobot(){
        //0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
        led_color_ = {{"green",0x42}, {"blue",0x52}, {"yellow",0x59}, {"red",0x47}, {"cyan",0x43}, {"white",0x57}, {"magenta",0x4D}};
        cmd1_.request.data = {0xB0,0x03,0x01,0x4D,0x00,0x00,0x00,0x00,0x03,0xE8,0x1B};
        cmd2_.request.data = {0xB0,0x01,0x02,0x4D,0x57,0x00,0x00,0x00,0x00,0x96,0x1B};
    }
    SetRobot::~SetRobot(){};

    bool SetRobot::setStatus(int status,std::string text){
        set_gobot_status_.request.status = status;
        set_gobot_status_.request.text = text;
        return ros::service::call("/gobot_status/set_gobot_status",set_gobot_status_);
    }

    bool SetRobot::setDock(int status){
        set_dock_status_.request.status = status;
        return ros::service::call("/gobot_status/set_dock_status",set_dock_status_);
    }

    bool SetRobot::setStage(const int stage){
        set_stage_.request.stage = stage;
        return ros::service::call("/gobot_status/set_stage", set_stage_);
    }

    bool SetRobot::setLoop(const int loop){
        set_loop_.request.data.push_back(loop);
        return ros::service::call("/gobot_status/set_loop",set_loop_);
    }

    bool SetRobot::setWifi(std::string wifi_name,std::string wifi_password){
        gobot_msg_srv::SetString set_wifi;
        set_wifi.request.data.push_back(wifi_name);
        set_wifi.request.data.push_back(wifi_password);
        return ros::service::call("/gobot_status/set_wifi",set_wifi); 
    }  

    bool SetRobot::setMute(const int mute){
        gobot_msg_srv::SetInt set_mute;
        set_mute.request.data.push_back(mute);
        return ros::service::call("/gobot_status/set_mute", set_mute);
    }

    bool SetRobot::setName(std::string robot_name){
        gobot_msg_srv::SetString set_name;
        set_name.request.data.push_back(robot_name);
        return ros::service::call("/gobot_status/set_name",set_name);
    }

    bool SetRobot::setBatteryLvl(std::string battery_lvl){
        gobot_msg_srv::SetString set_battery;
        set_battery.request.data.push_back(battery_lvl);
        return ros::service::call("/gobot_status/set_battery",set_battery);
    }

    bool SetRobot::setSpeed(std::string linear, std::string angular){
        gobot_msg_srv::SetString set_speed;
        set_speed.request.data.push_back(linear);
        set_speed.request.data.push_back(angular);
        return ros::service::call("/gobot_status/set_speed",set_speed);
    }

    bool SetRobot::setHome(std::string pos_x,std::string pos_y,std::string ori_x,std::string ori_y,std::string ori_z,std::string ori_w){
        gobot_msg_srv::SetString set_home;
        set_home.request.data.push_back(pos_x);
        set_home.request.data.push_back(pos_y);
        set_home.request.data.push_back(ori_x);
        set_home.request.data.push_back(ori_y);
        set_home.request.data.push_back(ori_z);
        set_home.request.data.push_back(ori_w);
        return ros::service::call("/gobot_status/set_home",set_home);
    }

    bool SetRobot::clearPath(){
        gobot_msg_srv::SetPath set_path;
        return ros::service::call("/gobot_status/set_path", set_path);
    }

    bool SetRobot::setMotorSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){ 
    motor_speed_.request.directionR = std::string(1, directionR);
    motor_speed_.request.velocityR = velocityR;
    motor_speed_.request.directionL = std::string(1, directionL);
    motor_speed_.request.velocityL = velocityL;

    return ros::service::call("/gobot_motor/setSpeeds", motor_speed_);
    }

    void SetRobot::setSound(int num,int time_on, int time_off){
        gobot_msg_srv::SetInt sound_num;
        sound_num.request.data.push_back(num);
        sound_num.request.data.push_back(time_on);
        if(time_off!=0)
            sound_num.request.data.push_back(time_off);

        ros::service::call("/gobot_base/setSound",sound_num);
    }
    
    void SetRobot::setBatteryLed(){
        ros::service::call("/gobot_base/show_Battery_LED",empty_srv);
    }

    void SetRobot::setPermanentLed(std::string color){
        cmd1_.request.data[3]=led_color_.at(color);
        ros::service::call("/gobot_base/setLed",cmd1_);
    }

    void SetRobot::setRunningLed(std::string color1, std::string color2){
        cmd2_.request.data[3]=led_color_.at(color1);
        cmd2_.request.data[4]=led_color_.at(color2);  //white
        ros::service::call("/gobot_base/setLed",cmd2_);
    }
};

