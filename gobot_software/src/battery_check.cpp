#include <gobot_software/battery_check.hpp>

bool canGoCharge = true;
int test = -1;
double LOW_BATTERY_THRESHOLD = 0.0;

/// For test purpose only
bool testAutoDocking(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res){
    ROS_INFO("(Battery check) Service testAutoDocking called");
    test = req.voltage;
    return true;
}

/// Check if the battery is low and the robot should go charge
void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    gobot_msg_srv::GetString get_battery;
    ros::service::call("/gobot_status/get_battery",get_battery);
    LOW_BATTERY_THRESHOLD = std::stod(get_battery.response.data[0]);

    if(!batteryInfo->ChargingFlag){
        if(canGoCharge && (batteryInfo->Percentage <= LOW_BATTERY_THRESHOLD || test == 0)){
            ROS_WARN("(Battery check) Battery info : %d %.2f %d. Battery is low, let's go charge!!", batteryInfo->BatteryVoltage, batteryInfo->Percentage,batteryInfo->ChargingCurrent);
            std_srvs::Empty arg;
            ros::service::call("/gobot_command/lowBattery", arg);
            canGoCharge = false;
        }
    }
    else{
        //reset the gocharge when complete godock
        if(batteryInfo->Percentage>0.75){
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
    
    ros::service::waitForService("/gobot_status/get_battery", ros::Duration(30));
    gobot_msg_srv::GetString get_battery;
    ros::service::call("/gobot_status/get_battery",get_battery);
    LOW_BATTERY_THRESHOLD = std::stod(get_battery.response.data[0]);

    ros::ServiceServer testAutoDockingService = nh.advertiseService("/gobot_test/testAutoDocking", testAutoDocking);

    ros::Subscriber batterySub = nh.subscribe("/gobot_base/battery_topic", 1, newBatteryInfo);
    
    ros::spin();

    return 0;
}