#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>
#include <gobot_msg_srv/BatteryMsg.h>
#include <gobot_msg_srv/SetBattery.h>
#include <gobot_msg_srv/GetString.h>
#include <gobot_msg_srv/GetInt.h>
#include <gobot_msg_srv/SetInt.h>
#include <gobot_msg_srv/IsCharging.h>

bool canGoCharge = true;
int monitor_state = -1, resume_work_level = 90;

//****************************** CALLBACK ******************************
void timerCallback(const ros::TimerEvent&){
    gobot_msg_srv::IsCharging isCharging;
    ros::service::call("/gobot_status/charging_status", isCharging);

    gobot_msg_srv::GetInt battery_percent;
    ros::service::call("/gobot_status/battery_percent", battery_percent);
    if((!isCharging.response.isCharging && canGoCharge) || monitor_state == 1){
        gobot_msg_srv::GetString get_battery;
        ros::service::call("/gobot_status/get_battery",get_battery);
        if((battery_percent.response.data<std::stoi(get_battery.response.data) || monitor_state == 1)){
            ROS_WARN("(BATTERY_MONITOR) Battery current percentage: %d, auto-charging percentage: %d ", battery_percent.response.data,std::stoi(get_battery.response.data));
            std_srvs::Empty arg;
            ros::service::call("/gobot_command/low_battery", arg);
            canGoCharge = false;
            monitor_state = 0;
        }
    }
    if(!canGoCharge || monitor_state == 2){
        if((battery_percent.response.data >= resume_work_level && isCharging.response.isCharging) || monitor_state == 2){
            ROS_WARN("(BATTERY_MONITOR) Battery current percentage:%d. Resume to work.", battery_percent.response.data);
            std_srvs::Empty arg;
            ros::service::call("/gobot_command/high_battery", arg);
            canGoCharge = true;
            monitor_state = 0;
        }
    }
}

//****************************** SERVICE ******************************
/// For test purpose only
bool testAutoDocking(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
    //1-low battery, 2-high battery
    monitor_state = req.data;
    ROS_INFO("(BATTERY_MONITOR) Service testAutoDocking called: %d", monitor_state);
    return true;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "battery_monitor_function");
    ros::NodeHandle nh;
    
    nh.getParam("resume_work_level", resume_work_level);
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(60.0));
    //Startup end

    ros::ServiceServer testAutoDockingService = nh.advertiseService("/gobot_test/testAutoDocking", testAutoDocking);

    ros::Timer battery_timer = nh.createTimer(ros::Duration(60), timerCallback);
    
    ros::spin();

    return 0;
}