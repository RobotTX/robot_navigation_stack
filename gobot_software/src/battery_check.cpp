#include <gobot_software/battery_check.hpp>

bool canGoCharge = true;
int test = -1;

/// For test purpose only
bool testAutoDocking(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res){
    ROS_INFO("(Battery check) Service testAutoDocking called");
    test = req.voltage;
    return true;
}

void timerCallback(const ros::TimerEvent&){
    gobot_msg_srv::IsCharging isCharging;
    ros::service::call("/gobot_status/charging_status", isCharging);

    gobot_msg_srv::GetInt battery_percent;
    ros::service::call("/gobot_status/battery_percent", battery_percent);

    if(!isCharging.response.isCharging){
           gobot_msg_srv::GetString get_battery;
           ros::service::call("/gobot_status/get_battery",get_battery);
        if(canGoCharge && (battery_percent.response.data[0] <= std::stod(get_battery.response.data[0]) || test == 0)){
            ROS_WARN("(Battery check) Battery below percentage: %d ", battery_percent.response.data[0]);
            std_srvs::Empty arg;
            ros::service::call("/gobot_command/lowBattery", arg);
            canGoCharge = false;
        }
    }
    else{
        //reset the gocharge when complete godock
        if(battery_percent.response.data[0]>75){
            canGoCharge = true;
        }
        else{
            canGoCharge = false;
        }
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "battery_check");
    ros::NodeHandle nh;
    ROS_INFO("(Battery check) running");
    
    ros::service::waitForService("/gobot_software/disconnet_servers", ros::Duration(30));

    ros::ServiceServer testAutoDockingService = nh.advertiseService("/gobot_test/testAutoDocking", testAutoDocking);

    ros::Timer battery_timer = nh.createTimer(ros::Duration(60), timerCallback);
    
    ros::spin();

    return 0;
}