#include <gobot_msg_srv/get_robot_class.h>

namespace robot_class {
    GetRobot::GetRobot(){};
    GetRobot::~GetRobot(){};

    void GetRobot::getStatus(int &status,std::string &text){
        ros::service::call("/gobot_status/get_gobot_status",get_gobot_status_);
        status = get_gobot_status_.response.status;
        text = get_gobot_status_.response.text;
    }

    void GetRobot::getStatus(int &status){
        ros::service::call("/gobot_status/get_gobot_status",get_gobot_status_);
        status = get_gobot_status_.response.status;
    }

    void GetRobot::getHome(double &x, double &y, double &qx, double &qy, double &qz, double &qw){
        gobot_msg_srv::GetStringArray get_home;
        ros::service::call("/gobot_status/get_home",get_home);
        x=std::stod(get_home.response.data[0]);
        y=std::stod(get_home.response.data[1]);
        qx=std::stod(get_home.response.data[2]);
        qy=std::stod(get_home.response.data[3]);
        qz=std::stod(get_home.response.data[4]);
        qw=std::stod(get_home.response.data[5]);
    }

    int GetRobot::getDock(){
        ros::service::call("/gobot_status/get_dock_status", get_dock_status_);
        return get_dock_status_.response.data;
    }

    int GetRobot::getLoop(){
        gobot_msg_srv::GetInt get_loop;
        ros::service::call("/gobot_status/get_loop",get_loop);
        return get_loop.response.data;
    }

    int GetRobot::getStage(){
        gobot_msg_srv::GetInt get_stage;
	    ros::service::call("/gobot_status/get_stage", get_stage);
        return get_stage.response.data;
    }

    std::string GetRobot::getName(){
        gobot_msg_srv::GetString get_name;
        ros::service::call("/gobot_status/get_name",get_name);
        return get_name.response.data;
    }
};

