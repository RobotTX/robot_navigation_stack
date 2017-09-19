#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <gobot_base/BatteryMsg.h>

bool canGoCharge = true;

void newBatteryInfo(const gobot_base::BatteryMsg::ConstPtr& batteryInfo){
    if(!batteryInfo->ChargingFlag && batteryInfo->BatteryVoltage < 18000 && canGoCharge){
        ROS_ERROR("(Battery check) Battery is low, let's go charge!!");
        std_srvs::Empty arg;
        if(ros::service::call("lowBattery", arg))
            canGoCharge = false;
        else
            ROS_ERROR("(Battery check) Could not call the lowBattery service");
    }
    if(!canGoCharge && batteryInfo->ChargingFlag)
        canGoCharge = true;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "battery_check");
    ros::NodeHandle nh;

    ROS_INFO("(Battery check) running");

    ros::Subscriber batterySub = nh.subscribe("/battery_topic", 1, newBatteryInfo);

    ros::spin();

    return 0;
}