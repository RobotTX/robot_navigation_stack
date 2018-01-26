#include "gobot_software/auto_docking.hpp"


#define PI 3.1415926

/// we want the robot to be at most at 0.15 metres of its goal
move_base_msgs::MoveBaseGoal currentGoal;
std::shared_ptr<MoveBaseClient> ac(0);

int attempt = 0;
bool docking = false;
bool collision = false;
bool lostIrSignal = false;
bool leftFlag = false;
bool charging = false;
bool move_from_collision = true;

ros::Time lastIrSignalTime;

ros::Subscriber goalStatusSub,bumperSub,irSub,batterySub,proximitySub,poseSub;
ros::Timer sound_timer;

std_srvs::Empty empty_srv;

tfScalar x, y, oriX, oriY, oriZ, oriW;
double landingPointX, landingPointY, landingYaw, currentYaw;

int dock_status = 0;
robot_class::SetRobot robot;

/****************************************** STEP 1 : Go 1.5 meters in front of the charging station *********************************************************/
/// Service to start docking
bool startDocking(void){
    ros::NodeHandle nh;

    ros::spinOnce();

    /// Get the charging station position from the home file
    gobot_msg_srv::GetString get_home;
    if(ros::service::call("/gobot_status/get_home",get_home)){
        x=std::stod(get_home.response.data[0]);
        y=std::stod(get_home.response.data[1]);
        oriX=std::stod(get_home.response.data[2]);
        oriY=std::stod(get_home.response.data[3]);
        oriZ=std::stod(get_home.response.data[4]);
        oriW=std::stod(get_home.response.data[5]);

        if(x != 0 || y != 0 || oriX != 0 || oriY != 0 || oriZ != 0){
            //~ROS_INFO("(auto_docking::startDocking) home found : [%f, %f] [%f, %f, %f, %f]", x, y, oriX, oriY, oriZ, oriW);

            landingYaw = tf::getYaw(tf::Quaternion(oriX , oriY , oriZ, oriW));

            /// We want to go 1 metre in front of the charging station
            landingPointX = x + 0.6 * std::cos(landingYaw);
            landingPointY = y + 0.6 * std::sin(landingYaw);
            //~ROS_INFO("(auto_docking::startDocking) landing point : [%f, %f, %f]", landingPointX, landingPointY, landingYaw);

            /// Create the goal
            currentGoal.target_pose.header.frame_id = "map";
            currentGoal.target_pose.header.stamp = ros::Time::now();
            currentGoal.target_pose.pose.position.x = landingPointX;
            currentGoal.target_pose.pose.position.y = landingPointY;
            currentGoal.target_pose.pose.position.z = 0;
            currentGoal.target_pose.pose.orientation.x = oriX;
            currentGoal.target_pose.pose.orientation.y = oriY;
            currentGoal.target_pose.pose.orientation.z = oriZ;
            currentGoal.target_pose.pose.orientation.w = oriW;
            
            /// send the goal
            if(ac->isServerConnected()) {
                startDockingParams();
                robot.setRunningLed("yellow","cyan");
                ac->sendGoal(currentGoal);

                return true;
            }
            else 
                ROS_ERROR("(auto_docking::startDocking) no action server");
        } 
        else
            ROS_ERROR("(auto_docking::startDocking) home is not valid (probably not set)");
    } 
    else
        ROS_ERROR("(auto_docking::startDocking) could not find the home pose");

    return false;
}

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if(docking){
        ROS_INFO("(auto_docking::newBumpersInfo) Goal status %d",msg->status.status);
        switch(msg->status.status){
			//SUCCEED
			case 3:
				/// if we reached the goal for the fisrt time
                findChargingStation();
				break;
			//OTHER CASE
			default:
                resetDockingParams();
                dock_status = -1;
                robot.setDock(dock_status);
                robot.setStatus(11,"FAIL_DOCKING");
                robot.setPermanentLed("red");
                robot.setSound(3,2);
                ROS_WARN("(auto_docking::finishedDocking) Auto docking finished->FAILED because goal can not be reached.");
				break;
		}
    }
}

/****************************************** STEP 2 : Use the IR to guide the robot to the charging station *********************************************************/
void findChargingStation(void){
    //~ROS_INFO("(auto_docking::findChargingStation) start to find charging station");
    ros::NodeHandle nh;
    goalStatusSub.shutdown();
    /// To check if we are charging
    //batterySub = nh.subscribe("/gobot_base/battery_topic", 1, newBatteryInfo);

    /// To check for collision
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, newBumpersInfo);

    /// Pid control with the ir signal
    lastIrSignalTime = ros::Time::now();
    irSub = nh.subscribe("/gobot_base/ir_topic", 1, newIrSignal);
    poseSub = nh.subscribe("/robot_pose",1,poseCallback);
}

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    /*
    /// if we are charging
    if(docking && !charging && batteryInfo->ChargingFlag){
        if(!collision){
            charging = true;
            irSub.shutdown();
            robot.setMotorSpeed('F', 0, 'F', 0);
            finishedDocking();
        }
    }
    */
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    if(docking){
        currentYaw = tf::getYaw(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
    }
}


void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    /// 0 : collision; 1 : no collision
    //at least two has no collision we will check
    if(docking){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        /// check if we have a collision
        if(back){
            if(!collision){
                irSub.shutdown();
                poseSub.shutdown();
                collision = true;
                move_from_collision = false;
                robot.setMotorSpeed('F', 0, 'F', 0);
                ROS_WARN("(auto_docking::newBumpersInfo) just got a new collision:%d,%d,%d,%d",bumpers->bumper5,bumpers->bumper6,bumpers->bumper7,bumpers->bumper8);
                //turn right
                if(bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1 && bumpers->bumper7==1)
                    robot.setMotorSpeed('B', 2, 'F', 3);
                //turn left
                else if(bumpers->bumper5==0 && bumpers->bumper6==1 && bumpers->bumper7==1 && bumpers->bumper8==1)
                    robot.setMotorSpeed('F', 3, 'B', 2);
                //turn a bit left
                else if(bumpers->bumper5==0 && bumpers->bumper6==0 && bumpers->bumper7==1 && bumpers->bumper8==1)
                    robot.setMotorSpeed('F', 3, 'B', 1);
                //turn a bit right
                else if(bumpers->bumper7==0 && bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1)
                    robot.setMotorSpeed('B', 1, 'F', 3);
                //forward
                else
                    robot.setMotorSpeed('F', 3, 'F', 3);
            }
        }
        else{ 
            if (!move_from_collision && collision){
                if(!charging){
                    move_from_collision = true;
                    robot.setMotorSpeed('F', 0, 'F', 0);
                    finishedDocking();
                }
            }
        }
    }
}

/// The pid control function
void newIrSignal(const gobot_msg_srv::IrMsg::ConstPtr& irSignal){
    /// if we are charging
    if(docking && !collision){
        if (irSignal->rearSignal != 0){
            lostIrSignal = false;
            /// rear ir received 1 and 2 signal, so robot goes backward
            if (irSignal->rearSignal == 3){
                /*
                double diffYaw = currentYaw-landingYaw;
                if(diffYaw>PI)
                    diffYaw = diffYaw - 2*PI;
                else if(diffYaw<-PI)
                    diffYaw = diffYaw + 2*PI;
                    
                ROS_INFO("Yaw Diff: %.3f", diffYaw);
                if((diffYaw<0.1 && diffYaw >0) || (diffYaw<0 && diffYaw>-0.1))   //0.1->5.7degree
                    robot.setMotorSpeed('B', 5, 'B', 5);
                else if(diffYaw>0.1)      //in the left of home yaw. R_VEL>L_VEL
                    robot.setMotorSpeed('B', 5, 'B', 4);
                else if(diffYaw<-0.1)  //in the right of home yaw   L_VEL>R_VEL
                    robot.setMotorSpeed('B', 4, 'B', 5);
                    */
                robot.setMotorSpeed('B', 5, 'B', 5);
            }
            else if (irSignal->rearSignal == 2)
                /// rear ir received signal 2, so robot turns right
                robot.setMotorSpeed('B', 4, 'F', 4);
            else if (irSignal->rearSignal == 1)
                /// rear ir received signal 1, so robot turns left
                robot.setMotorSpeed('F', 4, 'B', 4);
            
        }
        else if (irSignal->leftSignal != 0){
            /// received left signal
            leftFlag = true;
            if (irSignal->leftSignal == 3)
                robot.setMotorSpeed('B', 5, 'F', 5);
            else if (irSignal->leftSignal == 2)
                robot.setMotorSpeed('B', 5, 'F',10);
                /*
            else if (irSignal->leftSignal == 1)
                robot.setMotorSpeed('B', 10, 'F', 5);
                */

        } 
        else if (irSignal->rightSignal != 0){
            /// received right signal
            leftFlag = false;
            if (irSignal->rightSignal == 3)
                robot.setMotorSpeed('F', 5, 'B', 5);
            else if (irSignal->rightSignal == 2)
                robot.setMotorSpeed('F', 5, 'B', 10);
                /*
            else if (irSignal->rightSignal == 1)
                robot.setMotorSpeed('F',10, 'B', 5);
                */
        }

        if (irSignal->rearSignal == 0){
            if(!lostIrSignal){
                //ROS_WARN("(auto_docking::newIrSignal) just lost the ir signal");
                lostIrSignal = true;
                lastIrSignalTime = ros::Time::now();
                /// make the robot turn on itself
                if(leftFlag)
                    //if the left sensor is the last which saw the ir signal
                    robot.setMotorSpeed('B', 5, 'F', 5);
                else
                    robot.setMotorSpeed('F', 5, 'B', 5);
            } 
            else{
                /// if we lost the signal for more than 30 seconds, we failed docking, else, the robot should still be turning on itself
                if((ros::Time::now() - lastIrSignalTime).toSec() > 30.0){
                    robot.setMotorSpeed('F', 0, 'F', 0);
                    finishedDocking();
                }
            } 
        }
    }
}

/****************************************** STEP 3 : The robot is charging, so we align it with the charging station *********************************************************/

void newProximityInfo(const gobot_msg_srv::ProximityMsg::ConstPtr& proximitySignal){
    /*
    if(docking && !collision){
        //~ROS_INFO("(auto_docking::newProximityInfo) new proximity signal : %d %d", proximitySignal->signal1, proximitySignal->signal2);
        /// signal1 = leftSensor
        /// signal2 = rightSensor
        /// 0 : object; 1 : no object
        if(proximitySignal->signal1 && proximitySignal->signal2){
            /// we are charging but can't find the charging station on either of the signal (should not happen, tell the user to check)
            robot.setMotorSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(!proximitySignal->signal1 && !proximitySignal->signal2){
            /// we are charging and should be aligned
            robot.setMotorSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(!proximitySignal->signal1 && proximitySignal->signal2){
            /// left sensor ok, right sensor not ok
            //robot.setMotorSpeed('F', 1, 'B', 1);
            robot.setMotorSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(proximitySignal->signal1 && !proximitySignal->signal2){
            /// left sensor not ok, right sensor ok
            //robot.setMotorSpeed('B', 1, 'F', 1);
            robot.setMotorSpeed('F', 0, 'F', 0);
            finishedDocking();
        }
    }
    */
}

void finishedDocking(){
    ros::NodeHandle nh;
    gobot_msg_srv::IsCharging arg;
    resetDockingParams();
    ros::Duration(3.0).sleep();
    if(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging){
        dock_status = 1;
        robot.setStatus(11,"STOP_DOCKING");
        ROS_INFO("(auto_docking::finishedDocking) Auto docking finished->SUCESSFUL.");
        robot.setSound(1,2);
    }
    else{
        attempt++;
        if(attempt <= 3){
            ROS_WARN("(auto_docking::finishedDocking) Failed docking %d time(s)", attempt);
            robot.setMotorSpeed('F', 15, 'F', 15);
            ros::Duration(1.5).sleep();
            robot.setMotorSpeed('F', 0, 'F', 0);

            if(ac->isServerConnected()) {
                startDockingParams();
                if(attempt == 1){
                    currentGoal.target_pose.header.stamp = ros::Time::now();
                    currentGoal.target_pose.pose.position.x = landingPointX + 0.05*std::sin(landingYaw);
                    currentGoal.target_pose.pose.position.y = landingPointY - 0.05*std::cos(landingYaw);
                }
                else if(attempt == 2){
                    currentGoal.target_pose.header.stamp = ros::Time::now();
                    currentGoal.target_pose.pose.position.x = landingPointX - 0.05*std::sin(landingYaw);
                    currentGoal.target_pose.pose.position.y = landingPointY + 0.05*std::cos(landingYaw);
                }
                else {
                    currentGoal.target_pose.header.stamp = ros::Time::now();
                    currentGoal.target_pose.pose.position.x = landingPointX;
                    currentGoal.target_pose.pose.position.y = landingPointY;
                }
                ac->sendGoal(currentGoal);
            }
        }
        else{ 
            dock_status = -1;
            robot.setDock(dock_status);
            robot.setStatus(11,"FAIL_DOCKING");
            ROS_WARN("(auto_docking::finishedDocking) Auto docking finished->FAILED.");
            robot.setMotorSpeed('F', 15, 'F', 15);
            ros::Duration(2.0).sleep();
            robot.setMotorSpeed('F', 0, 'F', 0);
            robot.setPermanentLed("red");
            robot.setSound(3,2);
        }
    }
}

/***************************************************************************************************/
bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(dock_status==3){
        resetDockingParams();
        if(ac->isServerConnected()){
            robot.setPermanentLed("blue");
            ac->cancelAllGoals();
        }
        gobot_msg_srv::IsCharging arg;
        dock_status=(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging) ? 1 : 0; 
        robot.setDock(dock_status);
        robot.setStatus(11,"STOP_DOCKING");
    }
    return true;
}

bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    resetDockingParams();
    attempt = 0;
    return startDocking();
}


void timerCallback(const ros::TimerEvent&){
    if(docking){
        robot.setSound(2,1);
    }
}

void resetDockingParams(){
    robot.setMotorSpeed('F', 0, 'F', 0);
    bumperSub.shutdown();
    irSub.shutdown();
    poseSub.shutdown();
    //batterySub.shutdown();
    goalStatusSub.shutdown();
    charging = false;
    docking = false;
    collision = false;
    lostIrSignal = false;
    leftFlag = false;
    move_from_collision = true;
    sound_timer.stop();
}

void startDockingParams(){
    ros::NodeHandle nh;
    dock_status = 3;
    robot.setDock(dock_status);
    robot.setStatus(15,"DOCKING");
    docking = true;
    sound_timer.start();
    goalStatusSub = nh.subscribe("/move_base/result",1,goalResultCallback);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "auto_docking");
    ros::NodeHandle nh;

    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(60.0));
    
    sound_timer = nh.createTimer(ros::Duration(10), timerCallback);
    sound_timer.stop();

    ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));
    ac->waitForServer();

    ROS_INFO("(auto_docking::startDockingService) actionlib server ready!!");

    ros::ServiceServer startDockingSrv = nh.advertiseService("/gobot_function/startDocking", startDockingService);
    ros::ServiceServer stopDockingSrv = nh.advertiseService("/gobot_function/stopDocking", stopDockingService);

    ros::spin();
    
    return 0;
}