#include <gobot_control/battery_controller.hpp>


ros::Subscriber sub_leftBattery;
ros::Subscriber sub_rightBattery;

int16_t batteryStatus = 20;
int32_t batteryVoltage = 20;
int32_t chargingCurrent = 20;
int16_t temperature = 20;
int32_t fullCapacity = 100;

int32_t remainCapacity = 100;
bool chargingFlagLeft = false;
bool chargingFlagRight = false;
int nb = 20;

bool isChargingService(gobot_base::IsCharging::Request &req, gobot_base::IsCharging::Response &res){
    res.isCharging = (chargingFlagLeft & chargingFlagRight);

    return true;
}

void newLeftBattery(const gazebo_msgs::ContactsState::ConstPtr& msg){
    if(!chargingFlagLeft && msg->states.size() > 0)
        nb = 20;

    if(chargingFlagLeft && msg->states.size() == 0){
        if(nb == 0)
            chargingFlagLeft = (msg->states.size() > 0);
        else
            nb--;
    } else
        chargingFlagLeft = (msg->states.size() > 0);
}

void newRightBattery(const gazebo_msgs::ContactsState::ConstPtr& msg){
    if(!chargingFlagRight && msg->states.size() > 0)
        nb = 20;
    
    if(chargingFlagRight && msg->states.size() == 0){
        if(nb <= 0)
            chargingFlagRight = (msg->states.size() > 0);
        else
            nb--;
    } else
        chargingFlagRight = (msg->states.size() > 0);
}

bool setBattery(gobot_control::SetBattery::Request &req, gobot_control::SetBattery::Response &res){
    std::cout << "(battery_controller) Service setBattery called" << std::endl;
    remainCapacity = req.capacity;
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "battery_controller");
    std::cout << "(battery_controller) Ready to be launched." << std::endl;

    ros::NodeHandle n;
    
    ros::ServiceServer setBatteryService = n.advertiseService("setBattery", setBattery);
    ros::ServiceServer isChargingSrv = n.advertiseService("isCharging", isChargingService);

    ros::Publisher batteryPublisher = n.advertise<gobot_base::BatteryMsg>("battery_topic", 50);

    sub_leftBattery = n.subscribe("left_battery", 1, newLeftBattery);
    sub_rightBattery = n.subscribe("right_battery", 1, newRightBattery);

    std::cout << "(battery_controller) launched." << std::endl;

    gobot_base::BatteryMsg msg;
    msg.BatteryStatus = batteryStatus;
    msg.BatteryVoltage = batteryVoltage;
    msg.ChargingCurrent = chargingCurrent;
    msg.Temperature = temperature;
    msg.FullCapacity = fullCapacity;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        msg.RemainCapacity = remainCapacity;
        msg.ChargingFlag = (chargingFlagLeft & chargingFlagRight);
        batteryPublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
