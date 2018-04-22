#include <gobot_control/bumper_controller.hpp>

/// IR sensors
short int backLeftBumper = 0;
short int backLeftCylBumper = 0;
short int backRightBumper = 0;
short int backRightCylBumper = 0;
short int frontLeftBumper = 0;
short int frontLeftCylBumper = 0;
short int frontRightBumper = 0;
short int frontRightCylBumper = 0;

ros::Subscriber sub_backLeftBumper;
ros::Subscriber sub_backLeftCylBumper;
ros::Subscriber sub_backRightBumper;
ros::Subscriber sub_backRightCylBumper;
ros::Subscriber sub_frontLeftBumper;
ros::Subscriber sub_frontLeftCylBumper;
ros::Subscriber sub_frontRightBumper;
ros::Subscriber sub_frontRightCylBumper;

void newBackLeftBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    backLeftBumper = !(msg->states.size() > 0);
}

void newBackLeftCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    backLeftCylBumper = !(msg->states.size() > 0);
}

void newBackRightBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    backRightBumper = !(msg->states.size() > 0);
}

void newBackRightCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    backRightCylBumper = !(msg->states.size() > 0);
}

void newFrontLeftBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    frontLeftBumper = !(msg->states.size() > 0);
}

void newFrontLeftCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    frontLeftCylBumper = !(msg->states.size() > 0);
}

void newFrontRightBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    frontRightBumper = !(msg->states.size() > 0);
}

void newFrontRightCylBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
    frontRightCylBumper = !(msg->states.size() > 0);
}

bool poseReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  return true;
}

bool sensorsReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "bumper_controller");
    std::cout << "(bumper_controller) Ready to be launched." << std::endl;

    ros::NodeHandle n;
    
    sub_backLeftBumper = n.subscribe("back_left_bumper", 1, newBackLeftBumper);
    sub_backLeftCylBumper = n.subscribe("back_left_cyl_bumper", 1, newBackLeftCylBumper);
    sub_backRightBumper = n.subscribe("back_right_bumper", 1, newBackRightBumper);
    sub_backRightCylBumper = n.subscribe("back_right_cyl_bumper", 1, newBackRightCylBumper);
    sub_frontLeftBumper = n.subscribe("front_left_bumper", 1, newFrontLeftBumper);
    sub_frontLeftCylBumper = n.subscribe("front_left_cyl_bumper", 1, newFrontLeftCylBumper);
    sub_frontRightBumper = n.subscribe("front_right_bumper", 1, newFrontRightBumper);
    sub_frontRightCylBumper = n.subscribe("front_right_cyl_bumper", 1, newFrontRightCylBumper);

    ros::Publisher bumperPublisher = n.advertise<gobot_msg_srv::BumperMsg>("bumpers_topic", 50);
    gobot_msg_srv::BumperMsg msg;

    std::cout << "(bumper_controller) launched." << std::endl;

    //reset robot to original
    std::string lastPoseFile;
    double begin_x, begin_y;
    n.getParam("last_known_position_file", lastPoseFile);
    n.getParam("begin_x", begin_x);
    n.getParam("begin_y", begin_y);
    std::ofstream ofs(lastPoseFile, std::ofstream::out | std::ofstream::trunc);
    if(ofs.is_open()){
        ofs << begin_x << " " << begin_y << " " << 0 <<" "<< 0 <<" "<< 0 <<" "<< 1;
        ofs.close();
    } 
    
    ros::ServiceServer poseReadySrv = n.advertiseService("/gobot_startup/pose_ready", poseReadySrvCallback);
    ros::ServiceServer sensorsReadySrv = n.advertiseService("/gobot_startup/sensors_ready", sensorsReadySrvCallback);


    ros::Rate loop_rate(10);

    while(ros::ok()){
        msg.bumper1 = frontLeftCylBumper;
        msg.bumper2 = frontLeftBumper;
        msg.bumper3 = frontRightBumper;
        msg.bumper4 = frontRightCylBumper;
        msg.bumper5 = backLeftCylBumper;
        msg.bumper6 = backLeftBumper;
        msg.bumper7 = backRightBumper;
        msg.bumper8 = backRightCylBumper;

        bumperPublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
