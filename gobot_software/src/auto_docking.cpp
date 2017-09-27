#include "gobot_software/auto_docking.hpp"

/// we want the robot to be at most at 0.15 metres of its goal
#define ROBOT_POS_TOLERANCE 0.05

move_base_msgs::MoveBaseGoal currentGoal;
std::shared_ptr<MoveBaseClient> ac(0);

int attempt = 0;
int dockStatus = 0;
bool docking = false;
bool landingPointReached = false;
bool collision = false;
bool charging = false;
bool chargingFlag = false;
bool lostIrSignal = false;
bool leftFlag = false;

/// Used to wait for the new goal to be published so the status is reset and we don't use the status of the previous goal
bool newGoal = false;

std::chrono::system_clock::time_point collisionTime;
std::chrono::system_clock::time_point lastIrSignalTime;

ros::Subscriber goalStatusSub;
ros::Subscriber robotPoseSub;
ros::Subscriber bumperSub;
ros::Subscriber irSub;
ros::Subscriber batterySub;
ros::Subscriber proximitySub;

/****************************************** STEP 1 : Go 1.5 meters in front of the charging station *********************************************************/

/// Service to start docking
bool startDocking(void){
    ros::NodeHandle nh;

    docking = false;
    collision = false;
    charging = false;
    landingPointReached = false;
    lostIrSignal = false;
    leftFlag = false;
    newGoal = false;

    goalStatusSub.shutdown();
    robotPoseSub.shutdown();
    bumperSub.shutdown();
    irSub.shutdown();
    batterySub.shutdown();

    if(ac->isServerConnected())
        ac->cancelAllGoals();

    ros::spinOnce();

    /// Get the charging station position from the home file
    std::string homeFile;
    if(nh.hasParam("home_file")){
        nh.getParam("home_file", homeFile);
        ROS_INFO("(auto_docking::startDocking) home file path : %s", homeFile.c_str());
        std::ifstream ifs(homeFile);

        if(ifs.is_open()){
            tfScalar x, y, oriX, oriY, oriZ, oriW;
            ifs >> x >> y >> oriX >> oriY >> oriZ >> oriW;
            ifs.close();

            if(x != 0 || y != 0 || oriX != 0 || oriY != 0 || oriZ != 0 || oriW != 0){

                ROS_INFO("(auto_docking::startDocking) home found : [%f, %f] [%f, %f, %f, %f]", x, y, oriX, oriY, oriZ, oriW);

                /// Got a quaternion and want an orientation in radian
                tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(oriX , oriY , oriZ, oriW));

                tfScalar roll;
                tfScalar pitch;
                tfScalar yaw;

                matrix.getRPY(roll, pitch, yaw);
                double homeOri = -(yaw*180/3.14159);//-(orientation+90)*3.14159/180);

                /// We want to go 1 metre in front of the charging station
                double landingPointX = x + 1.0 * std::cos(yaw);
                double landingPointY = y + 1.0 * std::sin(yaw);
                ROS_INFO("(auto_docking::startDocking) landing point : [%f, %f, %f]", landingPointX, landingPointY, homeOri);

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
                    ac->sendGoal(currentGoal);

                    docking = true;

                    /// will allow us to check that we arrived at our destination
                    goalStatusSub = nh.subscribe("/move_base/status", 1, goalStatus);
                    robotPoseSub = nh.subscribe("/robot_pose", 1, newRobotPos);

                    ROS_INFO("(auto_docking::startDocking) service called successfully");

                    return true;
                } else 
                    ROS_ERROR("(auto_docking::startDocking) no action server");
            } else
                ROS_ERROR("(auto_docking::startDocking) home is not valid (probably not set)");
        } else
            ROS_ERROR("(auto_docking::startDocking) could not open the file %s", homeFile.c_str());
    } else
        ROS_ERROR("(auto_docking::startDocking) could not find the param home_file %s", homeFile.c_str());

    return false;
}

void newRobotPos(const geometry_msgs::Pose::ConstPtr& robotPos){
    /// if we are docking
    if(docking){
        /// we check if the robot is close enough to its goal
        if(std::abs(robotPos->position.x - currentGoal.target_pose.pose.position.x) < ROBOT_POS_TOLERANCE && 
           std::abs(robotPos->position.y - currentGoal.target_pose.pose.position.y) < ROBOT_POS_TOLERANCE){
            /// if the robot has already arrived, we want to wait for the next goal instead of repeating the same "success" functions
            if(!landingPointReached){
                ROS_INFO("(auto_docking::newRobotPos) newRobotPos robot close enough to the goal");
                ROS_INFO("(auto_docking::newRobotPos) robot position [%f, %f]", robotPos->position.x, robotPos->position.y);
                ROS_INFO("(auto_docking::newRobotPos) robot goal [%f, %f]", currentGoal.target_pose.pose.position.x, currentGoal.target_pose.pose.position.y);
                landingPointReached = true;
                findChargingStation();
            }
        }
    }
}

// to get the status of the robot (completion of the path towards its goal, SUCCEEDED = reached its goal, ACTIVE = currently moving towards its goal)
void goalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray){
    if(docking){
        if(goalStatusArray->status_list[0].status == actionlib_msgs::GoalStatus::SUCCEEDED){
            /// if we reached the goal for the fisrt time
            if(!landingPointReached && newGoal){
                landingPointReached = true;
                findChargingStation();
            }
        } else if(goalStatusArray->status_list[0].status == actionlib_msgs::GoalStatus::ABORTED 
            || goalStatusArray->status_list[0].status == actionlib_msgs::GoalStatus::REJECTED){
            /// if the goal could not be reached
            if(!landingPointReached && newGoal)
                failedDocking(-4);
            
        } else if(goalStatusArray->status_list[0].status == actionlib_msgs::GoalStatus::PENDING 
            || goalStatusArray->status_list[0].status == actionlib_msgs::GoalStatus::ACTIVE)
        	/// Wait for the new goal to be published so the status is reset and we don't use the status of the previous goal
            newGoal = true;
    }
}

/****************************************** STEP 2 : Use the IR to guide the robot to the charging station *********************************************************/


void findChargingStation(void){
    ROS_INFO("(auto_docking::findChargingStation) called");

    /// we don't need this subscriber anymore
    goalStatusSub.shutdown();
    robotPoseSub.shutdown();

    /// if action server is up -> cancel any goal
    if(ac->isServerConnected())
        ac->cancelAllGoals();

    ros::NodeHandle nh;

    /// To check if we are charging
    batterySub = nh.subscribe("/battery_topic", 1, newBatteryInfo);

    /// To check for collision
    bumperSub = nh.subscribe("/bumpers_topic", 1, newBumpersInfo);

    /// Pid control with the ir signal
    irSub = nh.subscribe("/ir_topic", 1, newIrSignal);
}

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    //ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR, velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("setSpeeds", speed);
}

void newBatteryInfo(const gobot_msg_srv::BatteryMsg::ConstPtr& batteryInfo){
    /// if we are charging
    if(docking && !charging && batteryInfo->ChargingFlag){
        charging = true;
        alignWithCS();
    }
    chargingFlag = batteryInfo->ChargingFlag;
}

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){

    /// 0 : collision; 1 : no collision
    bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);

    /// check if we have a collision
    if(back){
        /// if it's a new collision, we stop the robot
        if(!collision){
            ROS_WARN("(auto_docking::newBumpersInfo) just got a new collision");
            collision = true;
            collisionTime = std::chrono::system_clock::now();
            setSpeed('F', 0, 'F', 0);
        } else {
            /// if after 30 seconds, the obstacle is still there, we tell the user about the obstacle
            if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - collisionTime).count() > 30){
                failedDocking(-3);
            }
        }
    } else {
        /// if we had a collision and the obstacle left
        if(collision){
            ROS_INFO("(auto_docking::newBumpersInfo) the obstacle left after %f seconds", (double) (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - collisionTime).count() / 1000));
            setSpeed('F', 0, 'F', 0);
            collision = false;
        }
    }
}

/// The pid control function
void newIrSignal(const gobot_msg_srv::IrMsg::ConstPtr& irSignal){
    /// if we are charging
    if(docking && !collision){
        ROS_INFO("(auto_docking::newIrSignal) new ir signal : %d %d %d", irSignal->leftSignal, irSignal->rearSignal, irSignal->rightSignal);
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
                    setSpeed('B', 3, 'F', 3);
                else
                    setSpeed('F', 3, 'B', 3);
            } else
                /// if we lost the signal for more than 20 seconds, we failed docking, else, the robot should still be turning on itself
                if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - lastIrSignalTime).count() > 20)
                    failedDocking(-1);
        } else {
            /// we got an ir signal; 
            if(lostIrSignal){
                ROS_INFO("(auto_docking::newIrSignal) just retrieved the ir signal after %f seconds", (double) (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastIrSignalTime).count() / 1000));
                lostIrSignal = false;
            }

            if (irSignal->rearSignal != 0){
                /// rear ir received 1 and 2 signal, so robot goes backward
                if (irSignal->rearSignal == 3)
                    setSpeed('B', 3, 'B', 3);
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
                    setSpeed('B', 3, 'F', 3);
                else if (irSignal->leftSignal == 2)
                    setSpeed('B', 3, 'F', 5);
                else if (irSignal->leftSignal == 1)
                    setSpeed('B', 5, 'F', 3);

            } else if (irSignal->rightSignal != 0){
                /// received right signal
                leftFlag = false;
                if (irSignal->rightSignal == 3)
                    setSpeed('F', 3, 'B', 3);
                else if (irSignal->rightSignal == 2)
                    setSpeed('F', 3, 'B', 5);
                else if (irSignal->rightSignal == 1)
                    setSpeed('F', 5, 'B', 3);
            }
        }
    }
}

/****************************************** STEP 3 : The robot is charging, so we align it with the charging station *********************************************************/

void alignWithCS(void){
    ROS_INFO("(auto_docking::alignWithCS) The robot is charging, checking the alignment");

    bumperSub.shutdown();
    irSub.shutdown();

    ros::spinOnce();

    setSpeed('F', 0, 'F', 0);

    ros::NodeHandle nh;
    proximitySub = nh.subscribe("/proximity_topic", 1, newProximityInfo);
}

void newProximityInfo(const gobot_msg_srv::ProximityMsg::ConstPtr& proximitySignal){
    ROS_INFO("(auto_docking::newProximityInfo) new proximity signal : %d %d", proximitySignal->signal1, proximitySignal->signal2);
    /// signal1 = leftSensor
    /// signal2 = rightSensor
    /// 0 : object; 1 : no object
    if(proximitySignal->signal1 && proximitySignal->signal2){
        /// we are charging but can't find the charging station on either of the signal (should not happen, tell the user to check)
        setSpeed('F', 0, 'F', 0);
        finishedDocking(2);
    } else if(!proximitySignal->signal1 && !proximitySignal->signal2){
        /// we are charging and should be aligned
        setSpeed('F', 0, 'F', 0);
        finishedDocking(1);
    } else if(!proximitySignal->signal1 && proximitySignal->signal2)
        /// left sensor ok, right sensor not ok
        setSpeed('F', 2, 'B', 2);
    else if(proximitySignal->signal1 && !proximitySignal->signal2)
        /// left sensor not ok, right sensor ok
        setSpeed('B', 2, 'F', 2);
}

void finishedDocking(const int16_t status){
    ROS_INFO("(auto_docking::finishedDocking) Finished trying to dock with status %d", status);
    proximitySub.shutdown();

    /// TODO if simulation set battery voltage to 25000 => charged flag ?

    /// If the battery is still charging, we succesfully docked the robot
    if(chargingFlag)
        ROS_INFO("(auto_docking::finishedDocking) Finished docking and we are still charging");
    else
        ROS_WARN("(auto_docking::finishedDocking) Finished docking but we are not charging anymore.... oops");

    batterySub.shutdown();

    gobot_msg_srv::SetDockStatus dockStatus;
    dockStatus.request.status = status;

    ros::service::call("setDockStatus", dockStatus);
}

/***************************************************************************************************/

void failedDocking(const int status){
    attempt++;
    ROS_INFO("(auto_docking::failedDocking) failed docking %d time(s)", attempt);

    if(attempt <= 3)
        startDocking();
    else {
        finishedDocking(status);
        stopDocking();
    }
}

void stopDocking(void){
    ROS_INFO("(auto_docking::stopDocking) called");

    setSpeed('F', 0, 'F', 0);

    attempt = 0;
    docking = false;
    landingPointReached = false;
    collision = false;
    charging = false;
    lostIrSignal = false;

    /// if action server is up -> cancel
    if(ac->isServerConnected())
        ac->cancelAllGoals();

    /// unsubscribe so we don't receive messages for nothing
    goalStatusSub.shutdown();
    robotPoseSub.shutdown();
    bumperSub.shutdown();
    irSub.shutdown();
    batterySub.shutdown();
}

bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("(auto_docking::stopDockingService) service called");

    stopDocking();

    return true;
}

bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("(auto_docking::startDockingService) service called");

    docking = false;
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

    ros::ServiceServer startDockingSrv = nh.advertiseService("startDocking", startDockingService);
    ros::ServiceServer stopDockingSrv = nh.advertiseService("stopDocking", stopDockingService);

    ros::spin();
    
    return 0;
}