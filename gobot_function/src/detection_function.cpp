#include "gobot_function/detection_function.h"

/*
                       __FF__
                      |      |
                      |  CAM |     
                      |__  __|
                         BB
  + alignment data               - alignment data
 move to object frame          move to object frame 
   x-axis negative                x-axis positive
     ______                         ______
    [      ]                       [      ]
    [Object]                       [Object]
    [______]                       [______]

1pixel = 0.1cm = 0.001m in the distance of 0.75 meter


Alignment:  VALUE   COMMENT                 MOTION
            100     not detected            turn spot
            99      end alignment           ---
            0       aligned                 backward
            1       center in right         turn spot right slowest
            2       center in right         turn spot right slower
            5       center in right         turn spot right
            10      left area>right area    backward right wheel to turn right
            -1      center in left          turn spot left slowest
            -2      center in left          turn spot left slower
            -5      center in left          turn spot left
            -10     right area>left area    backward left wheel to turn left
*/

ros::Subscriber magnetSub, alignmentSub, bumperSub, goalStatusSub;
robot_class::RobotMoveClass MoveRobot;
ros::Time lastSignal;
std_srvs::Empty empty_srv;

//positive values -> turn right
//negative values -> turn left
bool left_turn_ = true, detection_on_ = false, y_flag_ = false, object_attached_ = false, rough_alignment_ = true;
int detection_search = 6, detection_base = 3, rough_threshold = 200;
robot_class::Point object_pose_;


//****************************** CALLBACK ******************************
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if(detection_on_){
        switch(msg->status.status){
            case 2:
                break;
			//SUCCEED
			case 3:
				/// rough_alignment when first reaching the starting pose
                if(rough_alignment_){
                    if(!roughAlignment())
                        startAlignObject();
                }
                else{
                    startAlignObject();
                }
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
            ROS_INFO("(OBJECT_DETECTION) Successfully fing object!");
            //set robot pose to be the object pose
            MoveRobot.setInitialpose(object_pose_.x,object_pose_.y,object_pose_.yaw);
            stopDetectionFunc("Successfully fing object!", "COMPLETE_TRACKING");
        }
    }
}

void bumpersCb(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    if(detection_on_){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        if(back){
            stopDetectionFunc("Bumpers are triggered!");
            MoveRobot.setMotorSpeed('F', 15, 'F', 15);
            ros::Duration(1.5).sleep();
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
                MoveRobot.turnRight(detection_search, 0);
            else
                MoveRobot.turnLeft(0, detection_search);
        }
        else{  
            switch(msg->data){
                case 100:
                    if(ros::Time::now()-lastSignal>ros::Duration(1.0)){
                        if(left_turn_){
                            MoveRobot.turnRight(detection_search);
                        }
                        else{
                            MoveRobot.turnLeft(detection_search);
                        }
                    }
                    break;

                //backward
                case 0:
                    MoveRobot.backward(detection_base);
                    lastSignal = ros::Time::now();
                    break;
                
                case 1:
                    //MoveRobot.turnRight(1);
                    MoveRobot.turnRight(2, 0);
                    break;
                
                case 2:
                    //MoveRobot.turnRight(2);
                    MoveRobot.turnRight(detection_base, 0);
                    break;
                
                case 5:
                    MoveRobot.turnRight(detection_base);
                    break;
                
                case 10:
                    MoveRobot.turnRight(detection_search, 0);
                    lastSignal = ros::Time::now();
                    break;
                
                //turn left
                case -1:
                    //MoveRobot.turnLeft(1);
                    MoveRobot.turnLeft(0, 2);
                    break;

                case -2:
                    //MoveRobot.turnLeft(2);
                    MoveRobot.turnLeft(0, detection_base);
                    break;
                
                case -5:
                    MoveRobot.turnLeft(detection_base);
                    break;
                
                case -10:
                    MoveRobot.turnLeft(0, detection_search);
                    lastSignal = ros::Time::now();
                    break;
                
                case 99:
                    ros::Duration(0.5).sleep();
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
    if(object_attached_){
        MoveRobot.setMagnet(false);
        object_attached_ = false;
        ros::Duration(1.0).sleep();
        MoveRobot.forward(15);
        ros::Duration(2.0).sleep();
        MoveRobot.stop();
    }

    object_pose_.yaw = std::stod(req.data[2]);
    object_pose_.x = std::stod(req.data[0]);
    object_pose_.y = std::stod(req.data[1]);

    robot_class::Point starting_pose;
    starting_pose.yaw =  object_pose_.yaw;
    starting_pose.x = object_pose_.x + 1.0*std::cos(object_pose_.yaw);
    starting_pose.y = object_pose_.y + 1.0*std::sin(object_pose_.yaw);

    MoveRobot.moveFromCS();

    MoveRobot.setStatus(16,"TRACKING");

    rough_alignment_ = true;
    detection_on_ = true;
    ros::NodeHandle nh;
    goalStatusSub = nh.subscribe("/move_base/result",1,goalResultCallback);
    MoveRobot.moveTo(starting_pose);
    
    ROS_INFO("(OBJECT_DETECTION) Start finding object");
    return true;
}

bool startDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    MoveRobot.setLed(1,{"green","blue"});

    //start camera capture
    ros::service::call("/usb_cam/start_capture",empty_srv);
    ros::Duration(2.0).sleep();

    startAlignObject();
    
    detection_on_ = true;
    return true;
}

bool stopDetectionCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
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
    MoveRobot.setMagnet(true);
    //start subscribe relevant topics
    ros::NodeHandle nh;
    alignmentSub = nh.subscribe("/gobot_function/object_alignment",1,alignmentCb);
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, bumpersCb);
    magnetSub = nh.subscribe("/gobot_base/magnet_topic", 1, magnetCb);

    ROS_INFO("(OBJECT_DETECTION) Start Align with Object");
}

void stopDetectionFunc(std::string result, std::string status_text){
    //stop all subscribers and camera capture to release computation load
    detection_on_ = false;

    alignmentSub.shutdown();
    magnetSub.shutdown();
    bumperSub.shutdown();
    goalStatusSub.shutdown();
    MoveRobot.stop();
    ros::service::call("/usb_cam/stop_capture",empty_srv);

    if(status_text!="COMPLETE_TRACKING"){
        //power off magnet to detach object
        MoveRobot.setMagnet(false);
        //move robot forward a bit to give some back between robot and attached object
        if(object_attached_){
            ros::Duration(1.0).sleep();
            MoveRobot.forward(15);
            ros::Duration(2.0).sleep();
            MoveRobot.stop();
            object_attached_ = false;
        }
    }
    else{
        object_attached_ = true;
        ros::service::call("/gobot_status/update_status",empty_srv);
    }
    MoveRobot.setStatus(12, status_text);
    ROS_WARN("(OBJECT_DETECTION) Mission end: %s", result.c_str());
}

bool roughAlignment(){
    rough_alignment_ = false;
    //start camera capture
    ros::service::call("/usb_cam/start_capture",empty_srv);
    //wait for a while to stabilize the camera
    ros::Duration(3.0).sleep();

    gobot_msg_srv::GetInt alignment_data;
    ros::service::call("/gobot_function/get_object_alignment",alignment_data);

    //check whether starting pose error is within the threshold
    robot_class::Point starting_pose;
    //if can not detect object in the current pose, re-navigate robot to the given pose
    if(alignment_data.response.data == 999){
        starting_pose.yaw =  object_pose_.yaw;
        starting_pose.x = object_pose_.x + 1.0*std::cos(object_pose_.yaw);
        starting_pose.y = object_pose_.y + 1.0*std::sin(object_pose_.yaw);
        MoveRobot.moveTo(starting_pose);
        return true;
    }
    //if can see object with big error, move horizontally along object frame x-axis
    else if(abs(alignment_data.response.data) > rough_threshold){
        gobot_msg_srv::GetStringArray robot_pose;
        ros::service::call("/gobot_status/get_pose",robot_pose);
        starting_pose.yaw = std::stod(robot_pose.response.data[2]);
        starting_pose.x = std::stod(robot_pose.response.data[0]) - std::sin(starting_pose.yaw)*alignment_data.response.data*0.001;
        starting_pose.y = std::stod(robot_pose.response.data[1]) + std::cos(starting_pose.yaw)*alignment_data.response.data*0.001;
        MoveRobot.moveTo(starting_pose);
        return true;
    }

    return false;
}

void mySigintHandler(int sig)
{   
    detection_on_ = false;
    MoveRobot.setMagnet(false);
    ros::shutdown();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "detection_function");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    
    //Startup begin
	//sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(60.0));
	ros::service::waitForService("/usb_cam/stop_capture",ros::Duration(60.0));
    //Startup end

    MoveRobot.moveClientInitialize();

    nh.getParam("detection_search", detection_search);
    nh.getParam("detection_base", detection_base);
    nh.getParam("rough_threshold", rough_threshold);

    ros::ServiceServer findObjectSrv = nh.advertiseService("/gobot_function/find_object", findObjectCb);
    ros::ServiceServer stopDetectionSrv = nh.advertiseService("/gobot_function/stop_detection", stopDetectionCb);

    ros::ServiceServer startDetectionSrv = nh.advertiseService("/gobot_test/start_detection", startDetectionCb);

    ros::Duration(1.0).sleep();
    ros::service::call("/usb_cam/stop_capture",empty_srv);

    std::cout<<"Search spd:"<<detection_search<<"; Backward spd:"<<detection_base<<"; Rough threshold:"<<rough_threshold<<std::endl;
    ros::spin();
    
    return 0;
}