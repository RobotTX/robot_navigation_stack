#include <gobot_control/proximity_controller.hpp>

/// IR sensors
short int leftSignal = 0;
short int rightSignal = 0;

ros::Subscriber sub_leftSignal;
ros::Subscriber sub_rightSignal;

void newLeftSignal(const sensor_msgs::Range::ConstPtr& msg){
    // according to the communication protocol specifications 
    leftSignal = (msg->range > 0.149);
}

void newRightSignal(const sensor_msgs::Range::ConstPtr& msg){
    // according to the communication protocol specifications 
    rightSignal = (msg->range > 0.149);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "proximity_controller");

    std::cout << "(proximity_controller) Ready to be launched." << std::endl;

    ros::NodeHandle n;
    
    sub_leftSignal = n.subscribe("/sensors/sonar_sensor/proximity_left", 1, newLeftSignal);
    sub_rightSignal = n.subscribe("/sensors/sonar_sensor/proximity_right", 1, newRightSignal);

    std::cout << "(proximity_controller) launched." << std::endl;


    ros::Publisher proximityPublisher = n.advertise<gobot_msg_srv::ProximityMsg>("proximity_topic", 50);
    gobot_msg_srv::ProximityMsg msg;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        msg.signal1 = leftSignal;
        msg.signal2 = rightSignal;
        
        proximityPublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
