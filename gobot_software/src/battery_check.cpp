#include <gobot_software/battery_check.hpp>

#define LOW_VOLTAGE 21500

bool canGoCharge = true;
int test = -1;

/// For test purpose only
bool testAutoDocking(gobot_msg_srv::SetBattery::Request &req, gobot_msg_srv::SetBattery::Response &res){
    ROS_INFO("(Battery check) Service testAutoDocking called");
    test = req.voltage;
    return true;
}

/// Check if the battery is low and the robot should go charge
void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){

    //ROS_INFO("(Battery check) Battery info : %d %d %d %d", batteryInfo->BatteryVoltage, batteryInfo->ChargingCurrent, batteryInfo->ChargingFlag, canGoCharge);

    if(!batteryInfo->ChargingFlag && (batteryInfo->BatteryVoltage < LOW_VOLTAGE || test == 0) && canGoCharge){
        ROS_INFO("(Battery check) Battery info : %d %d %d %d", batteryInfo->BatteryVoltage, batteryInfo->ChargingCurrent, batteryInfo->ChargingFlag, canGoCharge);
        ROS_ERROR("(Battery check) Battery is low, let's go charge!!");
        std_srvs::Empty arg;
        if(ros::service::call("/gobot_command/lowBattery", arg))
            canGoCharge = false;
        else
            ROS_WARN("(Battery check) Could not go docking, the robot might be busy with its path, will try again later");
    }
    
    //reset the gocharge when complete godock
    if(!canGoCharge && batteryInfo->ChargingFlag){
        ROS_INFO("(Battery check) Battery info : %d %d %d %d", batteryInfo->BatteryVoltage, batteryInfo->ChargingCurrent, batteryInfo->ChargingFlag, canGoCharge);
        ROS_INFO("(Battery check) Can go charge again!!");
        canGoCharge = true;
    }
    sleep(2);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "battery_check");
    ros::NodeHandle nh;

    ROS_INFO("(Battery check) running");

    ros::ServiceServer testAutoDockingService = nh.advertiseService("/gobot_test/testAutoDocking", testAutoDocking);

    ros::Subscriber batterySub = nh.subscribe("/gobot_base/battery_topic", 1, newBatteryInfo);

    ros::spin();

    return 0;
}