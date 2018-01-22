#include <gobot_msg_srv/set_status_class.h>


namespace gobot_class {
    SetStatus::SetStatus(){}

    bool SetStatus::setGobotStatus(int status,std::string text){
        set_gobot_status.request.status = status;
        set_gobot_status.request.text = text;
        return ros::service::call("/gobot_status/set_gobot_status",set_gobot_status);
    }

    bool SetStatus::setDockStatus(int status){
        set_dock_status.request.status = status;
        return ros::service::call("/gobot_status/set_dock_status",set_dock_status);
    }

    bool SetStatus::setStage(const int stage){
        set_stage.request.stage = stage;
        return ros::service::call("/gobot_status/set_stage", set_stage);
    }

    bool SetStatus::setLoop(const int loop){
        gobot_msg_srv::SetInt set_loop;
        set_loop.request.data.push_back(loop);
        return ros::service::call("/gobot_status/set_loop",set_loop);
    }

    bool SetStatus::setWifi(std::string wifi_name,std::string wifi_password){
        gobot_msg_srv::SetString set_wifi;
        set_wifi.request.data.push_back(wifi_name);
        set_wifi.request.data.push_back(wifi_password);
        return ros::service::call("/gobot_status/set_wifi",set_wifi); 
    }  

    bool SetStatus::setMute(const int mute){
        gobot_msg_srv::SetInt set_mute;
        set_mute.request.data.push_back(mute);
        return ros::service::call("/gobot_status/set_mute", set_mute);
    }

    bool SetStatus::setName(std::string robot_name){
        gobot_msg_srv::SetString set_name;
        set_name.request.data.push_back(robot_name);
        return ros::service::call("/gobot_status/set_name",set_name);
    }

    bool SetStatus::setBatteryLvl(std::string battery_lvl){
        gobot_msg_srv::SetString set_battery;
        set_battery.request.data.push_back(battery_lvl);
        return ros::service::call("/gobot_status/set_battery",set_battery);
    }

    bool SetStatus::setSpeed(std::string linear, std::string angular){
        gobot_msg_srv::SetString set_speed;
        set_speed.request.data.push_back(linear);
        set_speed.request.data.push_back(angular);
        return ros::service::call("/gobot_status/set_speed",set_speed);
    }

    bool SetStatus::setHome(std::string pos_x,std::string pos_y,std::string ori_x,std::string ori_y,std::string ori_z,std::string ori_w){
        gobot_msg_srv::SetString set_home;
        set_home.request.data.push_back(pos_x);
        set_home.request.data.push_back(pos_y);
        set_home.request.data.push_back(ori_x);
        set_home.request.data.push_back(ori_y);
        set_home.request.data.push_back(ori_z);
        set_home.request.data.push_back(ori_w);
        return ros::service::call("/gobot_status/set_home",set_home);
    }

    bool SetStatus::clearPath(){
        gobot_msg_srv::SetPath set_path;
        return ros::service::call("/gobot_status/set_path", set_path);
    }

    void SetStatus::setLed(std::string color){
        gobot_msg_srv::LedStrip cmd;
        cmd.request.data[0]=0xB0;
        cmd.request.data[1]=0x03;
        cmd.request.data[2]=0x01;
        cmd.request.data[3]=0x00;
        cmd.request.data[4]=0x00;
        cmd.request.data[5]=0x00;
        cmd.request.data[6]=0x00;
        cmd.request.data[7]=0x00;
        cmd.request.data[8]=0x03;
        cmd.request.data[9]=0xE8;
        cmd.request.data[10]=0x1B;
        //0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
        if (color == "yellow")
            cmd.request.data[3]=0x59;
        else if (color == "red")
            cmd.request.data[3]=0x47;
        else if (color == "blue")
            cmd.request.data[3]=0x52;
        else if (color == "green")
            cmd.request.data[3]=0x42;
        else
            cmd.request.data[3]=0x57;
        ros::service::call("/gobot_base/setLed",cmd);
    }

    void SetStatus::setSound(int num,int time_on, int time_off){
        gobot_msg_srv::SetInt sound_num;
        sound_num.request.data.push_back(num);
        sound_num.request.data.push_back(time_on);
        if(time_off!=0)
            sound_num.request.data.push_back(time_off);

        ros::service::call("/gobot_base/setSound",sound_num);
    }
};

