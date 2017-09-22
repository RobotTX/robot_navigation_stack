#include <gobot_control/sonar_controller.hpp>

int rearSignal = 0;
int leftSignal = 0;
int rightSignal = 0;
int frontLeftSignal = 0;
int frontRightSignal = 0;

ros::Subscriber sub_rearSignal;
ros::Subscriber sub_leftSignal;
ros::Subscriber sub_rightSignal;
ros::Subscriber sub_frontLeftSignal;
ros::Subscriber sub_frontRightSignal;


void newRearSignal(const sensor_msgs::Range::ConstPtr& msg){
    /// Meters to centimeters
    rearSignal = msg->range * 100;
}

void newLeftSignal(const sensor_msgs::Range::ConstPtr& msg){
    leftSignal = msg->range * 100;
}

void newRightSignal(const sensor_msgs::Range::ConstPtr& msg){
    rightSignal = msg->range * 100;
}

void newFrontLeftSignal(const sensor_msgs::Range::ConstPtr& msg){
    frontLeftSignal = msg->range * 100;
}

void newFrontRightSignal(const sensor_msgs::Range::ConstPtr& msg){
    frontRightSignal = msg->range * 100;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "sonar_controller");

    std::cout << "(sonar_controller) Ready to be launched." << std::endl;

    ros::NodeHandle n;
    
    sub_rearSignal = n.subscribe("/sensors/sonar_sensor/sonar_rear", 1, newRearSignal);
    sub_leftSignal = n.subscribe("/sensors/sonar_sensor/sonar_left", 1, newLeftSignal);
    sub_rightSignal = n.subscribe("/sensors/sonar_sensor/sonar_right", 1, newRightSignal);
    sub_frontLeftSignal = n.subscribe("/sensors/sonar_sensor/sonar_front_left", 1, newFrontLeftSignal);
    sub_frontRightSignal = n.subscribe("/sensors/sonar_sensor/sonar_front_right", 1, newFrontRightSignal);

    std::cout << "(sonar_controller) launched." << std::endl;


    ros::Publisher sonarPublisher = n.advertise<gobot_msg_srv::SonarMsg>("sonar_topic", 50);
    gobot_msg_srv::SonarMsg msg;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        msg.distance1 = rearSignal;
        msg.distance2 = frontRightSignal;
        msg.distance3 = frontLeftSignal;
        msg.distance4 = leftSignal;
        msg.distance5 = rightSignal;
        /// Top sonar not yet added to the model
        msg.distance6 = 0;
        /// Middle sonar not yet added to the model
        msg.distance7 = 0;
        
        sonarPublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
