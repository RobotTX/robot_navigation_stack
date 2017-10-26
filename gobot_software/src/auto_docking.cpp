#include "gobot_software/auto_docking.hpp"

/// we want the robot to be at most at 0.15 metres of its goal

move_base_msgs::MoveBaseGoal currentGoal;
std::shared_ptr<MoveBaseClient> ac(0);

int attempt = 0;
bool docking = false;
bool collision = false;
bool charging = false;
bool lostIrSignal = false;
bool leftFlag = false;
bool move_away_collision = true;

std::chrono::system_clock::time_point collisionTime;
std::chrono::system_clock::time_point lastIrSignalTime;

ros::Subscriber goalStatusSub;
ros::Subscriber bumperSub;
ros::Subscriber irSub;
ros::Subscriber batterySub;
ros::Subscriber proximitySub;
ros::ServiceClient setGobotStatusSrv,setDockStatusSrv;

std_srvs::Empty empty_srv;

gobot_msg_srv::SetGobotStatus set_gobot_status;
gobot_msg_srv::SetDockStatus set_dock_status;


/****************************************** STEP 1 : Go 1.5 meters in front of the charging station *********************************************************/

/// Service to start docking
bool startDocking(void){
    ros::NodeHandle nh;

    docking = false;
    collision = false;
    charging = false;
    lostIrSignal = false;
    move_away_collision = true;
    leftFlag = false;

    ros::spinOnce();

    /// Get the charging station position from the home file
    gobot_msg_srv::GetString get_home;
    if(ros::service::call("/gobot_status/get_home",get_home)){
        tfScalar x, y, oriX, oriY, oriZ, oriW;
        x=std::stod(get_home.response.data[0]);
        y=std::stod(get_home.response.data[1]);
        oriX=std::stod(get_home.response.data[2]);
        oriY=std::stod(get_home.response.data[3]);
        oriZ=std::stod(get_home.response.data[4]);
        oriW=std::stod(get_home.response.data[5]);

        if(x != 0 || y != 0 || oriX != 0 || oriY != 0 || oriZ != 0 || oriW != 0){

            //~ROS_INFO("(auto_docking::startDocking) home found : [%f, %f] [%f, %f, %f, %f]", x, y, oriX, oriY, oriZ, oriW);

            /// Got a quaternion and want an orientation in radian
            tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(oriX , oriY , oriZ, oriW));

            tfScalar roll;
            tfScalar pitch;
            tfScalar yaw;

            matrix.getRPY(roll, pitch, yaw);
            double homeOri = -(yaw*180/3.14159);//-(orientation+90)*3.14159/180);

            /// We want to go 1 metre in front of the charging station
            double landingPointX = x + 0.8 * std::cos(yaw);
            double landingPointY = y + 0.8 * std::sin(yaw);
            //~ROS_INFO("(auto_docking::startDocking) landing point : [%f, %f, %f]", landingPointX, landingPointY, homeOri);

            /// Create the goal
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
                set_gobot_status.request.status = 15;
                set_gobot_status.request.text = "DOCKING";
                setGobotStatusSrv.call(set_gobot_status);

                set_dock_status.request.status = 3;
                setDockStatusSrv.call(set_dock_status);

                ac->sendGoal(currentGoal);
                docking = true;

                /// will allow us to check that we arrived at our destination
                goalStatusSub = nh.subscribe("/move_base/result",1,goalResultCallback);

                //~ROS_INFO("(auto_docking::startDocking) service called successfully");

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
        switch(msg->status.status){
			//PREEMTED
			case 2:
				break;
			//SUCCEED
			case 3:
				/// if we reached the goal for the fisrt time
                findChargingStation();
				break;
			//ABORTED
			case 4:
				/// if the goal could not be reached
                finishedDocking();
                break;
			//REJECTED
			case 5:
                finishedDocking();
				break;
			//OTHER CASE
			default:
				ROS_ERROR("Unknown goal status %d",msg->status.status);
				break;
		}
    }
}

/****************************************** STEP 2 : Use the IR to guide the robot to the charging station *********************************************************/
void findChargingStation(void){
    //~ROS_INFO("(auto_docking::findChargingStation) start to find charging station");

    /// we don't need this subscriber anymore
    goalStatusSub.shutdown();

    ros::NodeHandle nh;

    /// To check if we are charging
    batterySub = nh.subscribe("/gobot_base/battery_topic", 1, newBatteryInfo);

    /// To check for collision
    bumperSub = nh.subscribe("/gobot_base/bumpers_topic", 1, newBumpersInfo);

    /// Pid control with the ir signal
    irSub = nh.subscribe("/gobot_base/ir_topic", 1, newIrSignal);
}

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    //ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR, velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("/gobot_motor/setSpeeds", speed);
}

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    /// if we are charging
    if(docking && !charging && batteryInfo->ChargingFlag){
        charging = true;
        irSub.shutdown();
        if(batteryInfo->Temperature==-1){
            ROS_WARN("(auto_docking::battery) Surge current. move a bit back before check alignment");
            setSpeed('B', 3, 'B', 3);
            ros::Duration(1.0).sleep();
            setSpeed('F', 0, 'F', 0);
        }
        if(move_away_collision)
            setSpeed('F', 0, 'F', 0);
        //~ROS_INFO("(auto_docking::battery) Checking the alignment");
        alignWithCS();
    }
}

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    /// 0 : collision; 1 : no collision
    //at least two has no collision we will check
    if((bumpers->bumper1+bumpers->bumper2+bumpers->bumper3+bumpers->bumper4+bumpers->bumper5+bumpers->bumper6+bumpers->bumper7+bumpers->bumper8)>1){
        bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);

        /// check if we have a collision
        if(back && docking && !collision){
            collision = true;
            move_away_collision=false;
            irSub.shutdown();
            setSpeed('F', 0, 'F', 0);
            ROS_WARN("(auto_docking::newBumpersInfo) just got a new collision:%d,%d,%d,%d",bumpers->bumper5,bumpers->bumper6,bumpers->bumper7,bumpers->bumper8);
            //turn right
            if(bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1 && bumpers->bumper7==1)
                setSpeed('B', 3, 'F', 3);
            //turn left
            else if(bumpers->bumper5==0 && bumpers->bumper6==1 && bumpers->bumper7==1 && bumpers->bumper8==1)
                setSpeed('F', 3, 'B', 3);
            //turn a bit left
            else if(bumpers->bumper5==0 && bumpers->bumper6==0 && bumpers->bumper7==1 && bumpers->bumper8==1)
                setSpeed('F', 3, 'F', 1);
            //turn a bit right
            else if(bumpers->bumper7==0 && bumpers->bumper8==0 && bumpers->bumper5==1 && bumpers->bumper6==1)
                setSpeed('F', 1, 'F', 3);
            //forward
            else
                setSpeed('F', 3, 'F', 3);
                
            std::thread([](){
                ros::Duration(1.0).sleep();
                setSpeed('F', 0, 'F', 0);
                collision = false;
            }).detach();
        }
        else if(!back && docking && !move_away_collision && !collision){
            move_away_collision = true;
            //~ROS_INFO("(auto_docking::newBumpersInfo) move away from collision");
            if(!charging){
                //~ROS_INFO("(auto_docking::bumper) Checking the alignment");
                alignWithCS();
            }
        }
    }
}

/// The pid control function
void newIrSignal(const gobot_msg_srv::IrMsg::ConstPtr& irSignal){
    /// if we are charging
    if(docking && !collision){
        //ROS_INFO("(auto_docking::newIrSignal) new ir signal : %d %d %d", irSignal->leftSignal, irSignal->rearSignal, irSignal->rightSignal);
        /// if we got no signal
        if(irSignal->rearSignal == 0 && irSignal->leftSignal == 0 && irSignal->rightSignal == 0){
            /// if we just lost the ir signal, we start the timer
            if(!lostIrSignal){
                ROS_WARN("(auto_docking::newIrSignal) just lost the ir signal");
                lostIrSignal = true;
                lastIrSignalTime = std::chrono::system_clock::now();
                /// make the robot turn on itself
                if(leftFlag)
                    //if the left sensor is the last which saw the ir signal
                    setSpeed('B', 5, 'F', 5);
                else
                    setSpeed('F', 5, 'B', 5);
            } else
                /// if we lost the signal for more than 20 seconds, we failed docking, else, the robot should still be turning on itself
                if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - lastIrSignalTime).count() > 20.0)
                    finishedDocking();
        } 
        else {
            /// we got an ir signal; 
            if(lostIrSignal){
                //~ROS_INFO("(auto_docking::newIrSignal) just retrieved the ir signal after %f seconds", (double) (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastIrSignalTime).count() / 1000));
                lostIrSignal = false;
            }

            if (irSignal->rearSignal != 0){
                /// rear ir received 1 and 2 signal, so robot goes backward
                if (irSignal->rearSignal == 3)
                    setSpeed('B', 5, 'B', 5);
                else if (irSignal->rearSignal == 2)
                    /// rear ir received signal 2, so robot turns right
                    setSpeed('B', 3, 'F', 3);
                else if (irSignal->rearSignal == 1)
                    /// rear ir received signal 1, so robot turns left
                    setSpeed('F', 3, 'B', 3);
                
            } else if (irSignal->leftSignal != 0){
                /// received left signal
                leftFlag = true;
                if (irSignal->leftSignal == 3)
                    setSpeed('B', 5, 'F', 5);
                else if (irSignal->leftSignal == 2)
                    setSpeed('B', 5, 'F',10);
                else if (irSignal->leftSignal == 1)
                    setSpeed('B', 10, 'F', 5);

            } else if (irSignal->rightSignal != 0){
                /// received right signal
                leftFlag = false;
                if (irSignal->rightSignal == 3)
                    setSpeed('F', 5, 'B', 5);
                else if (irSignal->rightSignal == 2)
                    setSpeed('F', 5, 'B', 10);
                else if (irSignal->rightSignal == 1)
                    setSpeed('F',10, 'B', 5);
            }
        }
    }
}

/****************************************** STEP 3 : The robot is charging, so we align it with the charging station *********************************************************/

void alignWithCS(void){
    charging = true;

    std::thread([](){
        //check collision in these 5 seconds
        ros::Time last_time=ros::Time::now();
        double dt=0.0;
        while(dt<6.0){
            dt=(ros::Time::now()-last_time).toSec();
            ros::Duration(0.2).sleep();
            ros::spinOnce();
        }
        bumperSub.shutdown();
        batterySub.shutdown();
        setSpeed('F', 0, 'F', 0);
        ros::NodeHandle nh;
        proximitySub = nh.subscribe("/gobot_base/proximity_topic", 1, newProximityInfo);
    }).detach();
}

void newProximityInfo(const gobot_msg_srv::ProximityMsg::ConstPtr& proximitySignal){
    if(docking && !collision){
        //~ROS_INFO("(auto_docking::newProximityInfo) new proximity signal : %d %d", proximitySignal->signal1, proximitySignal->signal2);
        /// signal1 = leftSensor
        /// signal2 = rightSensor
        /// 0 : object; 1 : no object
        if(proximitySignal->signal1 && proximitySignal->signal2){
            /// we are charging but can't find the charging station on either of the signal (should not happen, tell the user to check)
            setSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(!proximitySignal->signal1 && !proximitySignal->signal2){
            /// we are charging and should be aligned
            setSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(!proximitySignal->signal1 && proximitySignal->signal2){
            /// left sensor ok, right sensor not ok
            //setSpeed('F', 1, 'B', 1);
            setSpeed('F', 0, 'F', 0);
            finishedDocking();
        } else if(proximitySignal->signal1 && !proximitySignal->signal2){
            /// left sensor not ok, right sensor ok
            //setSpeed('B', 1, 'F', 1);
            setSpeed('F', 0, 'F', 0);
            finishedDocking();
        }
    }
}

void finishedDocking(){
    gobot_msg_srv::IsCharging arg;
    /// TODO if simulation set battery voltage to 25000 => charged flag ?
    if(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging){
        set_dock_status.request.status = 1;
        setDockStatusSrv.call(set_dock_status);
        ROS_INFO("(auto_docking::finishedDocking) Auto docking finished->SUCESSFUL.");
        stopDocking();
    }
    else{
        attempt++;
        if(attempt <= 3){
            ROS_WARN("(auto_docking::finishedDocking) Failed docking %d time(s)", attempt);
            stopDocking();
            startDocking();
        }
        else{ 
            set_dock_status.request.status = -1;
            setDockStatusSrv.call(set_dock_status);
            ROS_WARN("(auto_docking::finishedDocking) Auto docking finished->FAILED.");
            setSpeed('F', 20, 'F', 20);
            ros::Duration(1.0).sleep();
            stopDocking();
        }
    }
}

/***************************************************************************************************/

void stopDocking(void){
    if(set_dock_status.request.status!=3){
        set_gobot_status.request.status = 11;
        set_gobot_status.request.text = (set_dock_status.request.status==-1) ? "FAIL_DOCKING" : "STOP_DOCKING";
        setGobotStatusSrv.call(set_gobot_status);
    }

    docking=false;
    /// if action server is up -> cancel
    if(ac->isServerConnected() && set_dock_status.request.status!=1 && set_dock_status.request.status!=3)
        ac->cancelAllGoals();
    
    /// unsubscribe so we don't receive messages for nothing
    goalStatusSub.shutdown();
    bumperSub.shutdown();
    irSub.shutdown();
    batterySub.shutdown();
    proximitySub.shutdown();

    setSpeed('F', 0, 'F', 0);
    //~ROS_INFO("(auto_docking::stopDocking) called");
}

bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    gobot_msg_srv::IsCharging arg;
    if(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging){
        set_dock_status.request.status = 1;
        //~ROS_INFO("(auto_docking::stopDockingService) service called, Gobot is charging");
    }
    else{
        set_dock_status.request.status = 0;
        //~ROS_INFO("(auto_docking::stopDockingService) service called, Gobot is not charging");
    }
    setDockStatusSrv.call(set_dock_status);
    attempt = 0;
    stopDocking();

    return true;
}

bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //~ROS_INFO("(auto_docking::startDockingService) service called");
    attempt = 0;

    return startDocking();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "auto_docking");
    ros::NodeHandle nh;

    ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));
    ac->waitForServer();

    ROS_INFO("(auto_docking::startDockingService) actionlib server ready!!");

    currentGoal.target_pose.header.frame_id = "map";

    ros::ServiceServer startDockingSrv = nh.advertiseService("/gobot_function/startDocking", startDockingService);
    ros::ServiceServer stopDockingSrv = nh.advertiseService("/gobot_function/stopDocking", stopDockingService);

    setGobotStatusSrv = nh.serviceClient<gobot_msg_srv::SetGobotStatus>("/gobot_status/set_gobot_status");
    setDockStatusSrv = nh.serviceClient<gobot_msg_srv::SetDockStatus>("/gobot_status/set_dock_status");

    ros::spin();
    
    return 0;
}