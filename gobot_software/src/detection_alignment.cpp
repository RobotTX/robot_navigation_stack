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
#include <gobot_software/robot_move.h>


ros::Subscriber magnetSub, alignmentSub, bumperSub;
robot_class::SetRobot SetRobot;
ros::Time lastSignal;

//positive values -> turn right
//negative values -> turn left
bool left_turn_ = true, detection_on_ = false, detection_bingo_ = false;

void stopDetectionFunc(){
    alignmentSub.shutdown();
    magnetSub.shutdown();
    bumperSub.shutdown();
    detection_on_ = false;
    detection_bingo_ = false;
    left_turn_ = false;
    SetRobot.setNavSpeed('F', 0, 'F', 0);
}

void magnetCb(const std_msgs::Int8::ConstPtr& msg){
    if(detection_on_){
        if(msg->data == 1){
            ROS_INFO("Successfully fing object!");
            stopDetectionFunc();
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
            SetRobot.setLed(0,{"green"});
        }
    }
}

void bumpersCb(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    /*
    if(detection_on_){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        if(back){
            stopDetectionFunc();
            SetRobot.setMotorSpeed('F', 4, 'F', 4);
            ros::Duration(0.5).sleep();
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
        }
    }
    */
}


void alignmentCb(const std_msgs::Int16::ConstPtr& msg){
    if(detection_on_){
        int base_spd = abs(msg->data)>50 ? 5 : 3;
        int search_spd = 8, backward_spd = 2;
        
        if(abs(msg->data)>60)
            base_spd = 3;
        else if(abs(msg->data)>30)
            base_spd = 2;
        else
            base_spd = 1;

        if(msg->data == -1){
            if(ros::Time::now()-lastSignal>ros::Duration(1.0)){
                if(left_turn_)
                    SetRobot.setNavSpeed('B', search_spd, 'F', search_spd);
                else
                    SetRobot.setNavSpeed('F', search_spd, 'B', search_spd);
                }
            else{
                SetRobot.setNavSpeed('B', backward_spd, 'B', backward_spd);
            }
        }
        //backward
        else if(msg->data == 0){
            SetRobot.setNavSpeed('B', backward_spd, 'B', backward_spd);
            detection_bingo_ = true;
            lastSignal = ros::Time::now();
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

}

bool startDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ros::NodeHandle nh;
    alignmentSub = nh.subscribe("/object_alignment",1,alignmentCb);
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, bumpersCb);
    magnetSub = nh.subscribe("/gobot_base/magnet_topic", 1, magnetCb);
    SetRobot.setLed(1,{"blue","white"});
    gobot_msg_srv::SetBool magnet;
    magnet.request.data = true;
    ros::service::call("/gobot_base/set_magnet",magnet);
    detection_on_ = true;
    ROS_INFO("Start QR Code Detection");
    return true;
}

bool stopDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    gobot_msg_srv::SetBool magnet;
    magnet.request.data = false;
    ros::service::call("/gobot_base/set_magnet",magnet);
    stopDetectionFunc();
    SetRobot.setLed(0,{"red"});
    ROS_INFO("Stop QR Code Detection");
    return true;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "detection_alignment");
    ros::NodeHandle nh;

    SetRobot.initialize();
    RobotMove robot_m;
    //robot_m.initialize();

    ros::ServiceServer startDetectionSrv = nh.advertiseService("/gobot_function/startDetection", startDetectionCb);
    ros::ServiceServer stopDetectionSrv = nh.advertiseService("/gobot_function/stopDetection", stopDetectionCb);

    ros::spin();
    
    return 0;
}