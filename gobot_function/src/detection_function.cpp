#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <gobot_msg_srv/robot_move_class.h>
#include <gobot_msg_srv/get_robot_class.h>

ros::Subscriber magnetSub, alignmentSub, bumperSub;
robot_class::RobotMoveClass MoveRobot;
ros::Time lastSignal;

//positive values -> turn right
//negative values -> turn left
bool left_turn_ = true, detection_on_ = false;

void stopDetectionFunc(){
    alignmentSub.shutdown();
    magnetSub.shutdown();
    bumperSub.shutdown();
    detection_on_ = false;
    left_turn_ = false;
    MoveRobot.stop();
}

void magnetCb(const std_msgs::Int8::ConstPtr& msg){
    if(detection_on_){
        if(msg->data == 1){
            ROS_INFO("Successfully fing object!");
            stopDetectionFunc();
            MoveRobot.stop();
            MoveRobot.setLed(0,{"green"});
        }
    }
}

void bumpersCb(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    /*
    if(detection_on_){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        if(back){
            stopDetectionFunc();
            MoveRobot.setMotorSpeed('F', 4, 'F', 4);
            ros::Duration(0.5).sleep();
            MoveRobot.setMotorSpeed('F', 0, 'F', 0);
        }
    }
    */
}


void alignmentCb(const std_msgs::Int16::ConstPtr& msg){
    if(detection_on_){
        int base_spd = 2, search_spd = 6;

        if(msg->data == 0){
            if(ros::Time::now()-lastSignal>ros::Duration(1.0)){
                if(left_turn_)
                    MoveRobot.turnRight(search_spd);
                else
                    MoveRobot.turnLeft(search_spd);
            }
        }
        //backward
        else if(msg->data == 100){
            MoveRobot.backward(base_spd);
            lastSignal = ros::Time::now();
        }
        //turn right
        else if(msg->data == 1){
            MoveRobot.turnRight(1);
            left_turn_ = false;
        }
        else if(msg->data == 5){
            MoveRobot.turnRight(base_spd);
            left_turn_ = false;
        }
        else if(msg->data == 10){
            MoveRobot.turnRight(base_spd*2, 0);
            left_turn_ = false;
        }
        //turn left
        else if(msg->data == -1){
            MoveRobot.turnLeft(1);
            left_turn_ = true;
        }
        else if(msg->data == -5){
            MoveRobot.turnLeft(base_spd);
            left_turn_ = true;
        }
        else if(msg->data == -10){
            MoveRobot.turnLeft(0, base_spd*2);
            left_turn_ = true;
        }
    }

}

bool startDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ros::NodeHandle nh;
    alignmentSub = nh.subscribe("/object_alignment",1,alignmentCb);
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, bumpersCb);
    magnetSub = nh.subscribe("/gobot_base/magnet_topic", 1, magnetCb);
    MoveRobot.setLed(1,{"blue","white"});
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
    MoveRobot.setLed(0,{"red"});
    ROS_INFO("Stop QR Code Detection");
    return true;
}

void mySigintHandler(int sig)
{   
    gobot_msg_srv::SetBool magnet;
    magnet.request.data = false;
    ros::service::call("/gobot_base/set_magnet",magnet);
    ros::shutdown();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "detection_function");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    
    MoveRobot.moveClientInitialize();

    ros::ServiceServer startDetectionSrv = nh.advertiseService("/gobot_function/startDetection", startDetectionCb);
    ros::ServiceServer stopDetectionSrv = nh.advertiseService("/gobot_function/stopDetection", stopDetectionCb);

    ros::spin();
    
    return 0;
}