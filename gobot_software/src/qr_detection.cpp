#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>
#include <gobot_msg_srv/robot_msgs.h>


ros::Subscriber qrCodeSub, qrAlignmentSub, bumperSub;
robot_class::SetRobot SetRobot;


//positive values -> turn right
//negative values -> turn left
bool left_turn_ = true, detection_on = false, detection_bingo_ = false;

void stopDetectionFunc(){
    qrAlignmentSub.shutdown();
    qrCodeSub.shutdown();
    bumperSub.shutdown();
    detection_on = false;
    detection_bingo_ = false;
    left_turn_ = false;
    SetRobot.setNavSpeed('F', 0, 'F', 0);
}

void bumpersCb(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    if(detection_on){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        if(back){
            stopDetectionFunc();
            SetRobot.setMotorSpeed('F', 4, 'F', 4);
            ros::Duration(0.5).sleep();
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
        }
    }
}

void barcodeCb(const std_msgs::String::ConstPtr& msg){

}

void qrAlignmentCb(const std_msgs::Int16::ConstPtr& msg){
    int base_spd = abs(msg->data)>50 ? 4 : 2;
    int search_spd = 10;
    
    if(msg->data == -1){
        if(detection_bingo_)
            SetRobot.setNavSpeed('B', base_spd, 'B', base_spd);
        else if(left_turn_)
            SetRobot.setNavSpeed('B', search_spd, 'F', search_spd);
        else
            SetRobot.setNavSpeed('F', search_spd, 'B', search_spd);
    }
    //backward
    else if(msg->data == 0){
        SetRobot.setNavSpeed('B', base_spd, 'B', base_spd);
        detection_bingo_ = true;
    }
    //turn right
    else if(msg->data > 0){
        SetRobot.setNavSpeed('B', base_spd, 'F', base_spd);
        left_turn_ = false;
        detection_bingo_ = false;
    }
    //turn left
    else if(msg->data < 0){
        SetRobot.setNavSpeed('F', base_spd, 'B', base_spd);
        left_turn_ = true;
        detection_bingo_ = false;
    }

}

bool startDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ros::NodeHandle nh;
    qrCodeSub = nh.subscribe("barcode",1,barcodeCb);
    qrAlignmentSub = nh.subscribe("/qr_alignment",1,qrAlignmentCb);
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, bumpersCb);
    detection_on = true;
    ROS_INFO("Start QR Code Detection");
    return true;
}

bool stopDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    stopDetectionFunc();
    ROS_INFO("Stop QR Code Detection");
    return true;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "qr_detection");
    ros::NodeHandle nh;

    SetRobot.initialize();
    

    ros::ServiceServer startDetectionSrv = nh.advertiseService("/gobot_function/startDetection", startDetectionCb);
    ros::ServiceServer stopDetectionSrv = nh.advertiseService("/gobot_function/stopDetection", stopDetectionCb);

    ros::spin();
    
    return 0;
}