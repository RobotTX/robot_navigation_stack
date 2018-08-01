#include "gobot_function/docking_function.h"

int attempt = 0;
bool docking_on_ = false, collision = false, lostIrSignal = false, leftFlag = false, charging = false, move_from_collision = true;
tfScalar x, y, oriX, oriY, oriZ, oriW;
double homeX, homeY, homeYaw;

std_srvs::Empty empty_srv;

ros::Time lastIrSignalTime;

ros::Subscriber goalStatusSub,bumperSub,irSub,batterySub;

robot_class::RobotMoveClass MoveRobot;
robot_class::GetRobot GetRobot;

//****************************** CALLBACK ******************************
void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if(docking_on_){
        ROS_INFO("(AUTO_DOCKING::goalResultCallback) Goal status %d",msg->status.status);
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
                failedDocking("home can not be reached");
				break;
		}
    }
}


void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    /// if we are charging
    if(docking_on_ && !charging && batteryInfo->ChargingFlag){
        if(!collision){
            charging = true;
            irSub.shutdown();
            MoveRobot.stop();
            finishedDocking();
        }
    }
}


void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    int base_spd = 2;
    /// 0 : collision; 1 : no collision
    if(docking_on_){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);
        /// check if we have a collision
        if(back){
            irSub.shutdown();
            collision = true;
            move_from_collision = false;
            ROS_WARN("(AUTO_DOCKING::newBumpersInfo) Detect back collision:%d,%d,%d,%d",bumpers->bumper5,bumpers->bumper6,bumpers->bumper7,bumpers->bumper8);
            //turn right
            if(bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1 && bumpers->bumper7==1)
                MoveRobot.setMotorSpeed('B', base_spd*2, 'F', base_spd);
            //turn left
            else if(bumpers->bumper5==0 && bumpers->bumper6==1 && bumpers->bumper7==1 && bumpers->bumper8==1)
                MoveRobot.setMotorSpeed('F', base_spd, 'B', base_spd*2);
            //turn a bit left
            else if(bumpers->bumper5==0 && bumpers->bumper6==0 && bumpers->bumper7==1 && bumpers->bumper8==1)
                MoveRobot.setMotorSpeed('F', base_spd, 'B', 0);
            //turn a bit right
            else if(bumpers->bumper7==0 && bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1)
                MoveRobot.setMotorSpeed('B', 0, 'F', base_spd);
            //forward
            else
                MoveRobot.setMotorSpeed('F', base_spd, 'F', base_spd);
        }
        else{ 
            if (!move_from_collision && collision){
                if(!charging){
                    //move a bit front to leave space for bumper
                    move_from_collision = true;
                    MoveRobot.setMotorSpeed('F', 1, 'F', 1);
                    ros::Duration(0.5).sleep();
                    MoveRobot.setMotorSpeed('F', 0, 'F', 0);
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
    if(docking_on_ && !collision){
        if (irSignal->rearSignal != 0){
            lostIrSignal = false;
            /// rear ir received 1 and 2 signal, so robot goes backward
            if (irSignal->rearSignal == 3){
                MoveRobot.backward(base_spd);
            }
            else if (irSignal->rearSignal == 2){
                leftFlag = false;
                /// rear ir received signal 2, so robot turns left
                MoveRobot.turnLeft(0, base_spd);
            }
            else if (irSignal->rearSignal == 1){
                leftFlag = true;
                /// rear ir received signal 1, so robot turns right
                MoveRobot.turnRight(base_spd, 0);
            }
        }
        else if (irSignal->leftSignal != 0){
            /// received left signal
            leftFlag = true;
            if (irSignal->leftSignal == 3)
                MoveRobot.turnRight(base_spd);
            else if (irSignal->leftSignal == 2)
                MoveRobot.turnRight(base_spd, base_spd/2);
            else if (irSignal->leftSignal == 1)
                MoveRobot.turnRight(base_spd/2, base_spd);
        } 
        else if (irSignal->rightSignal != 0){
            /// received right signal
            leftFlag = false;
            if (irSignal->rightSignal == 3)
                MoveRobot.turnLeft(base_spd);
            else if (irSignal->rightSignal == 2)
                MoveRobot.turnLeft(base_spd, base_spd/2);
            else if (irSignal->rightSignal == 1)
                MoveRobot.turnLeft(base_spd/2, base_spd);
        }

        if (irSignal->rearSignal == 0){
            if(!lostIrSignal){
                //ROS_WARN("(AUTO_DOCKING::newIrSignal) just lost the ir signal");
                lostIrSignal = true;
                lastIrSignalTime = ros::Time::now();
                //make the robot turn on itself
                if(leftFlag)
                    MoveRobot.turnRight(base_spd);
                else
                    MoveRobot.turnLeft(base_spd);
            } 
            /// if we lost the rear signal for more than 20 seconds, we failed docking, else, the robot should still be turning on itself
            else if((ros::Time::now() - lastIrSignalTime) > ros::Duration(20.0)){
                MoveRobot.stop();
                finishedDocking(false);
            } 
        }
    }
}

//****************************** SERVICE ******************************
bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    resetDockingParams();
    attempt = 0;
    return startDocking();
}

bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    resetDockingParams();
    MoveRobot.cancelMove();
    
    gobot_msg_srv::IsCharging isCharging;
    ros::service::call("/gobot_status/charging_status", isCharging);
    if(!isCharging.response.isCharging){
        MoveRobot.setDock(0);
    }
    MoveRobot.setStatus(11,"STOP_DOCKING");
    return true;
}


//****************************** FUNCTIONS ******************************
bool startDocking(void){
    ros::NodeHandle nh;
    /// Get the charging station position from the home file
    GetRobot.getHome(x,y,oriX,oriY,oriZ,oriW);

    if(x != 0 || y != 0){
        /// We want to go 0.5 metre in front of the charging station
        homeYaw = tf::getYaw(tf::Quaternion(oriX , oriY , oriZ, oriW));
        homeX = x + 0.4 * std::cos(homeYaw);
        homeY = y + 0.4 * std::sin(homeYaw);
        //~ROS_INFO("(AUTO_DOCKING::startDocking) landing point : [%f, %f, %f]", homeX, homeY, homeYaw);

        startDockingParams();
        MoveRobot.moveTo(homeX, homeY, homeYaw);

        return true;
    } 
    else{
        failedDocking("home is not valid");
    }

    return false;
}

void findChargingStation(){
    ros::NodeHandle nh;
    goalStatusSub.shutdown();

    lostIrSignal = false;
    leftFlag = false;
    collision = false;
    charging = false;
    move_from_collision = true;
    lastIrSignalTime = ros::Time::now();
    /// To check if we are charging
    batterySub = nh.subscribe("/gobot_base/battery_topic", 1, newBatteryInfo);

    /// To check for collision
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, newBumpersInfo);
    /// Pid control with the ir signal
    irSub = nh.subscribe("/gobot_base/ir_topic", 1, newIrSignal);
}

void finishedDocking(bool move_forward){
    ros::NodeHandle nh;
    resetDockingParams();
    //wait for battery information
    ros::Duration(8.0).sleep();
    gobot_msg_srv::IsCharging isCharging;
    ros::service::call("/gobot_status/charging_status", isCharging);
    if(isCharging.response.isCharging){
        MoveRobot.setStatus(11,"COMPLETE_DOCKING");
        ROS_INFO("(AUTO_DOCKING::finishedDocking) Auto docking finished->SUCESSFUL.");
    }
    else{
        attempt++;
        if(attempt <= 3){
            ROS_WARN("(AUTO_DOCKING::finishedDocking) Failed docking %d time(s)", attempt);
            if(move_forward){
                MoveRobot.forward(15);
                ros::Duration(2.0).sleep();
                MoveRobot.stop();
            }

            startDockingParams();
            if(attempt == 1){
                //a bit right
                MoveRobot.moveTo(homeX+0.05*std::sin(homeYaw), homeY-0.05*std::cos(homeYaw), homeYaw);
            }
            else if(attempt == 2){
                //a bit left
                MoveRobot.moveTo(homeX-0.05*std::sin(homeYaw), homeY+0.05*std::cos(homeYaw), homeYaw);
            }
            else {
                MoveRobot.moveTo(homeX, homeY, homeYaw);
            }
        }
        else{ 
            failedDocking("after all attempts");
            if(move_forward){
                MoveRobot.forward(15);
                ros::Duration(2.0).sleep();
                MoveRobot.stop();
            }
        }
    }
}

void resetDockingParams(){
    docking_on_ = false;
    bumperSub.shutdown();
    irSub.shutdown();
    batterySub.shutdown();
    goalStatusSub.shutdown();
    charging = false;
    collision = false;
    move_from_collision = true;
    MoveRobot.stop();
}

void startDockingParams(){
    ros::NodeHandle nh;
    MoveRobot.setDock(3);
    MoveRobot.setStatus(15,"DOCKING");
    docking_on_ = true;
    goalStatusSub = nh.subscribe("/move_base/result",1,goalResultCallback);
}

void failedDocking(std::string reason){
    MoveRobot.setDock(-1);
    MoveRobot.setStatus(11,"FAIL_DOCKING");
    ROS_ERROR("(AUTO_DOCKING) Auto docking failed -> %s.",reason.c_str());
}

void mySigintHandler(int sig)
{   
    docking_on_ = false;
    ros::shutdown();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "docking_function");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(60.0));
    //Startup end

    MoveRobot.moveClientInitialize();

    ros::ServiceServer startDockingSrv = nh.advertiseService("/gobot_function/start_docking", startDockingService);
    ros::ServiceServer stopDockingSrv = nh.advertiseService("/gobot_function/stop_docking", stopDockingService);

    ros::spin();
    
    return 0;
}