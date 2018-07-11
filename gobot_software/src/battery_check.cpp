#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/SetBattery.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/IsCharging.h>

bool canGoCharge = true;
int test = -1;
int resume_work_level = 90;

/// For test purpose only
bool testAutoDocking(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    //1-low battery, 2-high battery
    test = req.data;
    ROS_INFO("(BATTERY_MONITOR) Service testAutoDocking called: %d", test);
    return true;
}

void timerCallback(const ros::TimerEvent&){
    gobot_msg_srv::IsCharging isCharging;
    ros::service::call("/gobot_status/charging_status", isCharging);

    gobot_msg_srv::GetInt battery_percent;
    ros::service::call("/gobot_status/battery_percent", battery_percent);
    if((!isCharging.response.isCharging && canGoCharge) || test == 1){
        gobot_msg_srv::GetString get_battery;
        ros::service::call("/gobot_status/get_battery",get_battery);
        if((battery_percent.response.data<std::stoi(get_battery.response.data) || test == 1)){
            ROS_WARN("(BATTERY_MONITOR) Battery current percentage: %d, auto-charging percentage: %d ", battery_percent.response.data,std::stoi(get_battery.response.data));
            ROS_WARN("(BATTERY_MONITOR) Arrange robot to go to charge.");
            std_srvs::Empty arg;
            ros::service::call("/gobot_command/lowBattery", arg);
            canGoCharge = false;
            test = 0;
        }
    }
    if(!canGoCharge || test == 2){
        if((battery_percent.response.data>resume_work_level && isCharging.response.isCharging) || test == 2){
            ROS_WARN("(BATTERY_MONITOR) Battery current percentage:%d. Resume to work.", battery_percent.response.data);
            std_srvs::Empty arg;
            ros::service::call("/gobot_command/highBattery", arg);
            canGoCharge = true;
            test = 0;
        }
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "battery_check");
    ros::NodeHandle nh;
    
    nh.getParam("resume_work_level", resume_work_level);
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    //Startup end

    ros::ServiceServer testAutoDockingService = nh.advertiseService("/gobot_test/testAutoDocking", testAutoDocking);

    ros::Timer battery_timer = nh.createTimer(ros::Duration(60), timerCallback);
    
    ros::spin();

    return 0;
}