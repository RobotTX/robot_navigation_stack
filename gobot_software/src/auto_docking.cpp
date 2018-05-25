#include "gobot_software/auto_docking.hpp"


#define PI 3.1415926

/// we want the robot to be at most at 0.15 metres of its goal
move_base_msgs::MoveBaseGoal currentGoal;
MoveBaseClient* ac;

int attempt = 0;
bool docking = false;
bool collision = false;
bool lostIrSignal = false;
bool leftFlag = false;
bool charging = false;
bool move_from_collision = true;

ros::Time lastIrSignalTime;

ros::Subscriber goalStatusSub,bumperSub,irSub,batterySub,proximitySub;

std_srvs::Empty empty_srv;

tfScalar x, y, oriX, oriY, oriZ, oriW;
double landingPointX, landingPointY, landingYaw;

int dock_status = 0;
robot_class::SetRobot SetRobot;
robot_class::GetRobot GetRobot;

/****************************************** STEP 1 : Go 1.5 meters in front of the charging station *********************************************************/
/// Service to start docking
bool startDocking(void){
    ros::NodeHandle nh;
    /// Get the charging station position from the home file
    GetRobot.getHome(x,y,oriX,oriY,oriZ,oriW);

    if(x != 0 || y != 0 || oriZ != 0){
        //~ROS_INFO("(AUTO_DOCKING::startDocking) home found : [%f, %f] [%f, %f, %f, %f]", x, y, oriX, oriY, oriZ, oriW);

        landingYaw = tf::getYaw(tf::Quaternion(oriX , oriY , oriZ, oriW));

        /// We want to go 1 metre in front of the charging station
        landingPointX = x + 0.5 * std::cos(landingYaw);
        landingPointY = y + 0.5 * std::sin(landingYaw);
        //~ROS_INFO("(AUTO_DOCKING::startDocking) landing point : [%f, %f, %f]", landingPointX, landingPointY, landingYaw);

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
            ac->sendGoal(currentGoal);
            startDockingParams();

            return true;
        }
    } 
    else{
        dock_status = -1;
        SetRobot.setDock(dock_status);
        SetRobot.setStatus(11,"FAIL_DOCKING");
        ROS_ERROR("(AUTO_DOCKING) Auto docking finished->FAILED because home is not valid");
    }

    return false;
}

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if(docking){
        ROS_INFO("(AUTO_DOCKING::newBumpersInfo) Goal status %d",msg->status.status);
        switch(msg->status.status){
            case 2:
                break;
			//SUCCEED
			case 3:
				/// if we reached the goal for the fisrt time
                findChargingStation();
				break;
			//OTHER CASE
			default:
                resetDockingParams();
                dock_status = -1;
                SetRobot.setDock(dock_status);
                SetRobot.setStatus(11,"FAIL_DOCKING");
                ROS_WARN("(AUTO_DOCKING) Auto docking finished->FAILED because goal can not be reached.");
				break;
		}
    }
}

/****************************************** STEP 2 : Use the IR to guide the robot to the charging station *********************************************************/
void findChargingStation(void){
    //~ROS_INFO("(AUTO_DOCKING::findChargingStation) start to find charging station");
    ros::NodeHandle nh;
    goalStatusSub.shutdown();
    /// To check if we are charging
    batterySub = nh.subscribe("/gobot_base/battery_topic", 1, newBatteryInfo);

    /// To check for collision
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, newBumpersInfo);

    /// Pid control with the ir signal
    irSub = nh.subscribe("/gobot_base/ir_topic", 1, newIrSignal);
    lastIrSignalTime = ros::Time::now();
}

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    /// if we are charging
    if(docking && !charging && batteryInfo->ChargingFlag){
        if(!collision){
            charging = true;
            irSub.shutdown();
            SetRobot.setNavSpeed('F', 0, 'F', 0);
            finishedDocking();
        }
    }
}

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    int base_spd = 2;
    /// 0 : collision; 1 : no collision
    if(docking){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        /// check if we have a collision
        if(back){
            irSub.shutdown();
            collision = true;
            move_from_collision = false;
            ROS_WARN("(AUTO_DOCKING::newBumpersInfo) Detect back collision:%d,%d,%d,%d",bumpers->bumper5,bumpers->bumper6,bumpers->bumper7,bumpers->bumper8);
            //turn right
            if(bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1 && bumpers->bumper7==1)
                SetRobot.setMotorSpeed('B', base_spd*2, 'F', base_spd);
            //turn left
            else if(bumpers->bumper5==0 && bumpers->bumper6==1 && bumpers->bumper7==1 && bumpers->bumper8==1)
                SetRobot.setMotorSpeed('F', base_spd, 'B', base_spd*2);
            //turn a bit left
            else if(bumpers->bumper5==0 && bumpers->bumper6==0 && bumpers->bumper7==1 && bumpers->bumper8==1)
                SetRobot.setMotorSpeed('F', base_spd, 'B', 0);
            //turn a bit right
            else if(bumpers->bumper7==0 && bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1)
                SetRobot.setMotorSpeed('B', 0, 'F', base_spd);
            //forward
            else
                SetRobot.setMotorSpeed('F', base_spd, 'F', base_spd);
        }
        else{ 
            if (!move_from_collision && collision){
                if(!charging){
                    move_from_collision = true;
                    SetRobot.setMotorSpeed('F', 2, 'F', 2);
                    ros::Duration(0.5).sleep();
                    SetRobot.setMotorSpeed('F', 0, 'F', 0);
                    finishedDocking();
                }
            }
        }
    }
}

/// The pid control functioncharging
void newIrSignal(const gobot_msg_srv::IrMsg::ConstPtr& irSignal){
    /// if we are charging
    int base_spd = 8;
    if(docking && !collision){
        if (irSignal->rearSignal != 0){
            lostIrSignal = false;
            /// rear ir received 1 and 2 signal, so robot goes backward
            if (irSignal->rearSignal == 3){
                SetRobot.setNavSpeed('B', base_spd, 'B', base_spd);
            }
            else if (irSignal->rearSignal == 2){
                leftFlag = false;
                /// rear ir received signal 2, so robot turns left
                SetRobot.setNavSpeed('F', 0, 'B', base_spd);
            }
            else if (irSignal->rearSignal == 1){
                leftFlag = true;
                /// rear ir received signal 1, so robot turns right
                SetRobot.setNavSpeed('B', base_spd, 'F', 0);
            }
        }
        else if (irSignal->leftSignal != 0){
            /// received left signal
            leftFlag = true;
            if (irSignal->leftSignal == 3)
                SetRobot.setNavSpeed('B', base_spd, 'F', base_spd);
            else if (irSignal->leftSignal == 2)
                SetRobot.setNavSpeed('B', base_spd, 'F', base_spd/2);
            else if (irSignal->leftSignal == 1)
                SetRobot.setNavSpeed('B', base_spd/2, 'F', base_spd);

        } 
        else if (irSignal->rightSignal != 0){
            /// received right signal
            leftFlag = false;
            if (irSignal->rightSignal == 3)
                SetRobot.setNavSpeed('F', base_spd, 'B', base_spd);
            else if (irSignal->rightSignal == 2)
                SetRobot.setNavSpeed('F', base_spd, 'B', base_spd/2);
            else if (irSignal->rightSignal == 1)
                SetRobot.setNavSpeed('F', base_spd/2, 'B', base_spd);
        }

        if (irSignal->rearSignal == 0){
            if(!lostIrSignal){
                //ROS_WARN("(AUTO_DOCKING::newIrSignal) just lost the ir signal");
                lostIrSignal = true;
                lastIrSignalTime = ros::Time::now();
                /// make the robot turn on itself
                if(leftFlag)
                    //if the left sensor is the last which saw the ir signal
                    SetRobot.setNavSpeed('B', base_spd, 'F', base_spd);
                else
                    SetRobot.setNavSpeed('F', base_spd, 'B', base_spd);
            } 
            /// if we lost the rear signal for more than 20 seconds, we failed docking, else, the robot should still be turning on itself
            else if((ros::Time::now() - lastIrSignalTime) > ros::Duration(20.0)){
                SetRobot.setNavSpeed('F', 0, 'F', 0);
                finishedDocking(false);
            } 
        }
    }
}

/****************************************** STEP 3 : The robot is charging, so we align it with the charging station *********************************************************/

void newProximityInfo(const gobot_msg_srv::ProximityMsg::ConstPtr& proximitySignal){
    /*
    if(docking && !collision){
        //~ROS_INFO("(AUTO_DOCKING::newProximityInfo) new proximity signal : %d %d", proximitySignal->signal1, proximitySignal->signal2);
        /// signal1 = leftSensor
        /// signal2 = rightSensor
        /// 0 : object; 1 : no object
        if(proximitySignal->signal1 && proximitySignal->signal2){
            /// we are charging but can't find the charging station on either of the signal (should not happen, tell the user to check)
            SetRobot.setNavSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(!proximitySignal->signal1 && !proximitySignal->signal2){
            /// we are charging and should be aligned
            SetRobot.setNavSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(!proximitySignal->signal1 && proximitySignal->signal2){
            /// left sensor ok, right sensor not ok
            //SetRobot.setNavSpeed('F', 1, 'B', 1);
            SetRobot.setNavSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(proximitySignal->signal1 && !proximitySignal->signal2){
            /// left sensor not ok, right sensor ok
            //SetRobot.setNavSpeed('B', 1, 'F', 1);
            SetRobot.setNavSpeed('F', 0, 'F', 0);
            finishedDocking();
        }
    }
    */
}

void finishedDocking(bool move_forward){
    ros::NodeHandle nh;
    resetDockingParams();
    ros::Duration(6.0).sleep();
    gobot_msg_srv::IsCharging isCharging;
    if(ros::service::call("/gobot_status/charging_status", isCharging) && isCharging.response.isCharging){
        dock_status = 1;
        SetRobot.setStatus(11,"COMPLETE_DOCKING");
        ROS_INFO("(AUTO_DOCKING) Auto docking finished->SUCESSFUL.");
    }
    else{
        attempt++;
        if(attempt <= 3){
            ROS_WARN("(AUTO_DOCKING) Failed docking %d time(s)", attempt);
            if(move_forward){
                SetRobot.setNavSpeed('F', 15, 'F', 15);
                ros::Duration(1.5).sleep();
                SetRobot.setNavSpeed('F', 0, 'F', 0);
            }

            if(ac->isServerConnected()) {
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
                startDockingParams();
            }
        }
        else{ 
            dock_status = -1;
            SetRobot.setDock(dock_status);
            SetRobot.setStatus(11,"FAIL_DOCKING");
            ROS_WARN("(AUTO_DOCKING) Auto docking finished->FAILED.");
            if(move_forward){
                SetRobot.setNavSpeed('F', 15, 'F', 15);
                ros::Duration(2.0).sleep();
                SetRobot.setNavSpeed('F', 0, 'F', 0);
            }
        }
    }
}

/***************************************************************************************************/
bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    resetDockingParams();
    if(ac->isServerConnected())
        ac->cancelAllGoals();
    
    gobot_msg_srv::IsCharging isCharging;
    if(ros::service::call("/gobot_status/charging_status", isCharging) && isCharging.response.isCharging){
        dock_status= 1; 
    }
    else{
        dock_status= 0; 
        SetRobot.setDock(dock_status);
    }
    SetRobot.setStatus(11,"STOP_DOCKING");
    return true;
}

bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    resetDockingParams();
    attempt = 0;
    return startDocking();
}


void resetDockingParams(){
    bumperSub.shutdown();
    irSub.shutdown();
    batterySub.shutdown();
    goalStatusSub.shutdown();
    charging = false;
    docking = false;
    collision = false;
    leftFlag = false;
    move_from_collision = true;
    lostIrSignal = false;
    SetRobot.setNavSpeed('F', 0, 'F', 0);
}

void startDockingParams(){
    ros::NodeHandle nh;
    dock_status = 3;
    SetRobot.setDock(dock_status);
    SetRobot.setStatus(15,"DOCKING");
    docking = true;
    goalStatusSub = nh.subscribe("/move_base/result",1,goalResultCallback);
}

void mySigintHandler(int sig)
{   goalStatusSub.shutdown();
    bumperSub.shutdown();
    irSub.shutdown();
    batterySub.shutdown();
    proximitySub.shutdown();
	delete ac;
    ros::shutdown();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "auto_docking");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    
    SetRobot.initialize();
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    //Startup end
    
    ac = new MoveBaseClient("move_base", true);
    ac->waitForServer(ros::Duration(60.0));

    ros::ServiceServer startDockingSrv = nh.advertiseService("/gobot_function/startDocking", startDockingService);
    ros::ServiceServer stopDockingSrv = nh.advertiseService("/gobot_function/stopDocking", stopDockingService);

    ros::spin();
    
    return 0;
}