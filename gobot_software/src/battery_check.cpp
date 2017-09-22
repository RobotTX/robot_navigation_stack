#include <gobot_software/battery_check.hpp>

bool canGoCharge = true;

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){

    ROS_INFO("(Battery check) Battery info : %d %d %d", batteryInfo->BatteryVoltage, batteryInfo->ChargingFlag, canGoCharge);

    if(!batteryInfo->ChargingFlag && batteryInfo->BatteryVoltage < 18000 && canGoCharge){
        ROS_ERROR("(Battery check) Battery is low, let's go charge!!");
        std_srvs::Empty arg;
        if(ros::service::call("lowBattery", arg))
            canGoCharge = false;
        else
            ROS_WARN("(Battery check) Could not go docking, the robot might be busy with its path, will try again later");
    }
    if(!canGoCharge && batteryInfo->ChargingFlag){
        ROS_INFO("(Battery check) Can go charge again!!");
        canGoCharge = true;
    }

    sleep(1);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "battery_check");
    ros::NodeHandle nh;

    ROS_INFO("(Battery check) running");

    ros::Subscriber batterySub = nh.subscribe("/battery_topic", 1, newBatteryInfo);

    ros::spin();

    return 0;
}