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

    bool SetStatus::updatePath(){
        return ros::service::call("/gobot_function/update_path", empty_srv);
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

