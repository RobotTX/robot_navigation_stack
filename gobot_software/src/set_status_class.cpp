#include <gobot_software/set_status_class.h>

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

void SetStatus::setSound(int num,int time_on, int time_off){
    gobot_msg_srv::SetInt sound_num;
    sound_num.request.data.push_back(num);
    sound_num.request.data.push_back(time_on);
    if(time_off!=0)
        sound_num.request.data.push_back(time_off);

    ros::service::call("/gobot_base/setSound",sound_num);
}

