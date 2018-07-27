#include "gobot_function/detection_function.h"

ros::Subscriber magnetSub, alignmentSub, bumperSub, goalStatusSub;
robot_class::RobotMoveClass MoveRobot;
ros::Time lastSignal;

//positive values -> turn right
//negative values -> turn left
bool left_turn_ = true, detection_on_ = false, y_flag_ = false;
int BASE_SPD = 3, SEARCH_SPD = 6, BACKWARD_SPD = 3;



//****************************** CALLBACK ******************************
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if(detection_on_){
        switch(msg->status.status){
            case 2:
                break;
			//SUCCEED
			case 3:
				/// if we reached the goal for the fisrt time
                startAlignObject();
				break;
			//OTHER CASE
			default:
                stopDetectionFunc("Can not reach object starting pose!", "FAIL_TRACKING");
				break;
		}
    }
}

void magnetCb(const std_msgs::Int8::ConstPtr& msg){
    if(detection_on_){
        if(msg->data == 1){
            ros::NodeHandle n;
            ROS_INFO("(OBJECT_DETECTION) Successfully fing object!");
            stopDetectionFunc("Successfully fing object!", "COMPLETE_TRACKING");
        }
    }
}

void bumpersCb(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    if(detection_on_){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        if(back){
            stopDetectionFunc("Bumpers are triggered!");
            MoveRobot.setMotorSpeed('F', BASE_SPD, 'F', BASE_SPD);
            ros::Duration(1.0).sleep();
            MoveRobot.setMotorSpeed('F', 0, 'F', 0);
        }
    }
}


void alignmentCb(const std_msgs::Int16::ConstPtr& msg){
    if(detection_on_){
        if (ros::Time::now()-lastSignal>ros::Duration(30.0)){
            stopDetectionFunc("Can not find object!", "FAIL_TRACKING");
            return;
        }

        if(y_flag_ && abs(msg->data)<=2)
            y_flag_ = false;
            
        if(y_flag_){
            if(left_turn_)
                MoveRobot.turnRight(SEARCH_SPD, 0);
            else
                MoveRobot.turnLeft(0, SEARCH_SPD);
        }
        else{  
            switch(msg->data){
                case 100:
                    if(ros::Time::now()-lastSignal>ros::Duration(1.0)){
                        if(left_turn_){
                            MoveRobot.turnRight(SEARCH_SPD);
                        }
                        else{
                            MoveRobot.turnLeft(SEARCH_SPD);
                        }
                    }
                    break;

                //backward
                case 0:
                    MoveRobot.backward(BACKWARD_SPD);
                    lastSignal = ros::Time::now();
                    break;
                
                case 1:
                    MoveRobot.turnRight(1);
                    break;
                
                case 2:
                    MoveRobot.turnRight(2);
                    break;
                
                case 5:
                    MoveRobot.turnRight(BASE_SPD);
                    break;
                
                case 10:
                    MoveRobot.turnRight(SEARCH_SPD, 0);
                    break;
                
                //turn left
                case -1:
                    MoveRobot.turnLeft(1);
                    break;

                case -2:
                    MoveRobot.turnLeft(2);
                    break;
                
                case -5:
                    MoveRobot.turnLeft(BASE_SPD);
                    break;
                
                case -10:
                    MoveRobot.turnLeft(0, SEARCH_SPD);
                    break;
                
                case 99:
                    MoveRobot.stop();
                    y_flag_ = true;
                    lastSignal = ros::Time::now();
                    break;
                
                default:
                    break;
            }

            if(msg->data != 99 && msg->data != 100){
                left_turn_ = msg->data>0 ? false : true;
            }
        }
    }

}

//****************************** SERVICE ******************************
bool findObjectCb(gobot_msg_srv::SetStringArray::Request &req, gobot_msg_srv::SetStringArray::Response &res){
    //power off magnet to detach object before starting new attachment
    gobot_msg_srv::SetBool magnet;
    magnet.request.data = false;
    ros::service::call("/gobot_base/set_magnet",magnet);

    robot_class::Point object_pose;
    object_pose.yaw = std::stod(req.data[2]);
    object_pose.x = std::stod(req.data[0]) + 1.0*std::cos(object_pose.yaw);
    object_pose.y = std::stod(req.data[1]) + 1.0*std::sin(object_pose.yaw);

    MoveRobot.moveFromCS();

    MoveRobot.setStatus(16,"TRACKING");

    detection_on_ = true;
    ros::NodeHandle nh;
    goalStatusSub = nh.subscribe("/move_base/result",1,goalResultCallback);
    MoveRobot.moveTo(object_pose);
    
    ROS_INFO("(OBJECT_DETECTION) Start finding object");
    return true;
}

bool startDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    MoveRobot.setLed(1,{"green","blue"});

    startAlignObject();
    
    detection_on_ = true;
    return true;
}

bool stopDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //power off magnet to detach object
    gobot_msg_srv::SetBool magnet;
    magnet.request.data = false;
    ros::service::call("/gobot_base/set_magnet",magnet);
    MoveRobot.cancelMove();
    stopDetectionFunc("Stop detection service called!");

    ROS_INFO("(OBJECT_DETECTION) Stop Object Detection");
    return true;
}

//****************************** FUNCTIONS ******************************
void startAlignObject(){
    goalStatusSub.shutdown();
    y_flag_ = false;
    left_turn_ = true;
    lastSignal = ros::Time::now();
    //power on magnet to get ready for attach object
    gobot_msg_srv::SetBool magnet;
    magnet.request.data = true;
    ros::service::call("/gobot_base/set_magnet",magnet);
    ros::NodeHandle nh;
    alignmentSub = nh.subscribe("/gobot_function/object_alignment",1,alignmentCb);
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, bumpersCb);
    magnetSub = nh.subscribe("/gobot_base/magnet_topic", 1, magnetCb);

    ROS_INFO("(OBJECT_DETECTION) Start Align with Object");
}

void stopDetectionFunc(std::string result, std::string status_text){
    detection_on_ = false;
    alignmentSub.shutdown();
    magnetSub.shutdown();
    bumperSub.shutdown();
    goalStatusSub.shutdown();
    MoveRobot.stop();
    MoveRobot.setStatus(12, status_text);
    ROS_WARN("(OBJECT_DETECTION) Mission end: %s", result.c_str());
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

    nh.getParam("SEARCH_SPD", SEARCH_SPD);
    nh.getParam("BASE_SPD", BASE_SPD);
    nh.getParam("BACKWARD_SPD", BACKWARD_SPD);
    
    ros::ServiceServer findObjectSrv = nh.advertiseService("/gobot_function/find_object", findObjectCb);
    ros::ServiceServer stopDetectionSrv = nh.advertiseService("/gobot_function/stop_detection", stopDetectionCb);

    ros::ServiceServer startDetectionSrv = nh.advertiseService("/gobot_test/start_detection", startDetectionCb);

    ros::spin();
    
    return 0;
}