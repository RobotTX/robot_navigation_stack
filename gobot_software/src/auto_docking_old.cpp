#include "auto_docking_old.hpp"

/// we want the robot to be at most at 0.15 metres of its goal
#define ROBOT_POS_TOLERANCE 0.15

move_base_msgs::MoveBaseGoal currentGoal;
std::shared_ptr<MoveBaseClient> ac(0);

bool docking = false;
bool goalReached = false;

bool leftFlag = false;
bool rightFlag = false;
bool rearFlag = false;

int bumperCrashCount = 0;
std::chrono::system_clock::time_point beginTime;

ros::Subscriber goalStatusSubscriber;
ros::Subscriber robotPoseSubscriber;

/****************************************** STEP 1 : Go 1.5 meters in front of the charging station *********************************************************/

bool startDocking(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ros::NodeHandle nh;

    docking = false;
    goalReached = false;

    leftFlag = false;
    rightFlag = false;
    rearFlag = false;

    bumperCrashCount = 0;

    /// Get the charging station position from the home file
    std::string homeFile;
    if(nh.hasParam("home_file")){
        nh.getParam("home_file", homeFile);
        std::cout << "(auto_docking::startDocking) home file path : " << homeFile << std::endl;
        std::ifstream ifs(homeFile);

        if(ifs.is_open()){
            tfScalar x, y, oriX, oriY, oriZ, oriW;
            ifs >> x >> y >> oriX >> oriY >> oriZ >> oriW;
            ifs.close();

            std::cout << "(auto_docking::startDocking) home found : " << x << " " << y << " " << oriX << " " << oriY << " " << oriZ << " " << oriW << std::endl;

            /// Got a quaternion and want an orientation in radian
            tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(oriX , oriY , oriZ, oriW));

            tfScalar roll;
            tfScalar pitch;
            tfScalar yaw;

            matrix.getRPY(roll, pitch, yaw);
            double homeOri = -(yaw*180/3.14159);//-(orientation+90)*3.14159/180);

            /// We want to go 1.5 metres in front of the charging station
            double landingPointX = x + 1.5 * std::cos(homeOri);
            double landingPointY = y + 1.5 * std::sin(homeOri);
            std::cout << "(auto_docking::startDocking) landing point : " << landingPointX << " " << landingPointY << " " << homeOri << std::endl;

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
                goalStatusSubscriber = nh.subscribe("/move_base/status", 1, goalStatus);
                robotPoseSubscriber = nh.subscribe("/robot_pose", 1, getRobotPos);

                std::cout << "(auto_docking::startDocking) service called succesfully" << std::endl;

                return true;
            } else 
                std::cout << "(auto_docking::startDocking) no action server" << std::endl;
        } else
            std::cout << "(auto_docking::startDocking) could not open the file " << homeFile << std::endl;
    } else
        std::cout << "(auto_docking::startDocking) could not find the param home_file " << homeFile << std::endl;

    return false;
}

void getRobotPos(const geometry_msgs::Pose::ConstPtr& robotPos){
    /// if we are docking
    if(docking){
        /// we check if the robot is close enough to its goal
        if(std::abs(robotPos->position.x - currentGoal.target_pose.pose.position.x) < ROBOT_POS_TOLERANCE && std::abs(robotPos->position.y - currentGoal.target_pose.pose.position.y) < ROBOT_POS_TOLERANCE){
            /// if the robot has already arrived, we want to wait for the next goal instead of repeating the same "success" functions
            if(!goalReached){
                std::cout << "(auto_docking::getRobotPos) getRobotPos robot close enough to the goal" << std::endl;
                std::cout << "(auto_docking::getRobotPos) robot position " << robotPos->position.x << " " << robotPos->position.y << std::endl;
                std::cout << "(auto_docking::getRobotPos) robot goal " << currentGoal.target_pose.pose.position.x << " " << currentGoal.target_pose.pose.position.y << std::endl;
                goalReached = true;
                findChargingStation();
            }
        }
    }
}

// to get the status of the robot (completion of the path towards its goal, SUCCEEDED = reached its goal, ACTIVE = currently moving towards its goal)
void goalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray){
    if(docking){
        if(goalStatusArray->status_list[0].status == 3){
            /// if we reached the goal
            if(!goalReached){
                goalReached = true;
                findChargingStation();
            }
        } else if(goalStatusArray->status_list[0].status == 4 || goalStatusArray->status_list[0].status == 5){
            /// if the goal could not be reached
            if(!goalReached){
                stopDocking();
                /// TODO try a few times more and if it's still not working, tell command_system or the desktop application that we stopped trying to dock
            }
        }
        /// NOTE not sure it's useful
        /* else {
            goalReached = false;
        }*/
    }
}

/****************************************** STEP 2 : Use the IR to guide the robot to the charging station *********************************************************/


void findChargingStation(void){
    std::cout << "(auto_docking::findChargingStation) called" << std::endl;

    /// unsubscribe so we don't receive these messages for nothing
    goalStatusSubscriber.shutdown();
    robotPoseSubscriber.shutdown();

    /// if action server is up -> cancel any goal
    if(ac->isServerConnected())
        ac->cancelAllGoals();

    int docked = 0;

    ros::Rate r(10);
    while(ros::ok() && docked == 0){

        /// Get the battery informations
        sonar::GetBatteryInfo batteryInfo;
        if(ros::service::call("getBatteryInfo", batteryInfo)){
            //std::cout << "(auto_docking::findChargingStation) getBatteryInfo service called with success" << std::endl;

            /// If the battery is charging, we succesfully docked the robot
            if(batteryInfo.response.ChargingFlag){
                docked = 1;

                /// We stop the robot
                setSpeed('F', 0, 'F', 0);
                std::cout << "(auto_docking::findChargingStation) Charging station found" << std::endl;
            } else {
                docked = pidControl();
            }
        } else {
            std::cout << "(auto_docking::findChargingStation) getBatteryInfo service call failed" << std::endl;
            docked = -1;
        }

        ros::spinOnce();
        r.sleep();
    }

    if(docked == 1){
        /// TODO send a message to command_system or the desktop application
        std::cout << "(auto_docking::findChargingStation) succesfull docking" << std::endl;
    } else {
        std::cout << "(auto_docking::findChargingStation) could not dock, trying again" << std::endl;

        std_srvs::Empty arg;
        if(!ros::service::call("startDocking", arg))
            /// TODO send a message to command_system or the desktop application that we stopped trying to dock
            std::cout << "(auto_docking::findChargingStation) could not call the docking service" << std::endl;
    }
}

bool pidControl(void){
    /// Get the sensors informations
    sonar::GetIrSignal irSignal;
    sonar::GetShortSignal shortSignal;
    sonar::GetBumpers bumpers;

    if(ros::service::call("getIrSignal", irSignal)){
        int16_t rearSignal = irSignal.response.rearSignal;
        int16_t leftSignal = irSignal.response.leftSignal;
        int16_t rightSignal = irSignal.response.rightSignal;

        if(ros::service::call("getShortSignal", shortSignal)){
            int16_t shortSignal1 = shortSignal.response.signal1;
            int16_t shortSignal2 = shortSignal.response.signal2;

            if(ros::service::call("getBumpers", bumpers)){
                //std::cout << "(auto_docking::findChargingStation) all services called with success" << std::endl;
                /// Guide the robot to the charging station
                int16_t bumperValue = bumpers.response.values[4] + bumpers.response.values[5] + bumpers.response.values[6] + bumpers.response.values[7];


                std::cout << "(auto_docking::pidControl) " << rearSignal << " " << leftSignal << " " << rightSignal << " " 
                << shortSignal1 << " " << shortSignal2 << " " << bumperValue << " " << rearFlag << " " << leftFlag << " " << rightFlag << std::endl;

                /// 1: no collision; 0: collision
                if (bumperValue<4)
                    bumperCrashCount = bumperCrashCount + 1; ///Calculate the number of collisions
                if (bumperCrashCount == 0)
                    beginTime = std::chrono::system_clock::now(); ///start the timer

                if (shortSignal1 == 0 && shortSignal2 == 0)
                    rearFlag = false;
                else
                    rearFlag = true;
                
                if (bumperCrashCount == 0){  /// no collision
                    if (rearSignal == 0 && leftSignal == 0 && rightSignal == 0){
                        /// we got no ir signal
                        if (!leftFlag && !rightFlag){
                            if (rearFlag)
                                setSpeed('B', 3, 'F', 3);
                            else
                                setSpeed('B', 3, 'B', 3);
                        }
                    } else {
                        /// received ir signal
                        if (rearSignal != 0){
                            /// received rear signal
                            leftFlag = false;
                            rightFlag = false;
                            if (rearFlag){
                                if (rearSignal == 3)
                                    setSpeed('B', 3, 'B', 3);
                                    /// rear ir received 1 and 2 signal, so robot goes backward
                                else if (rearSignal == 2)
                                    setSpeed('F', 3, 'B', 3);
                                    /// rear ir received 2 signal, so robot turns right
                                else if (rearSignal == 1)
                                    setSpeed('B', 3, 'F', 3);
                                    /// rear ir received 1 signal, so robot turns left
                            } else
                                setSpeed('B',4, 'B', 4);
                                /// NOTE ???? check too close => too far
                                /// The robot is too close to the charging station, so the robot go backward
                        }
                        if (leftSignal != 0){
                            /// received left signal
                            leftFlag = true;
                            if (leftSignal == 1)
                                setSpeed('F', 3, 'B', 5);
                            else if (leftSignal == 2)
                                setSpeed('F', 5, 'B', 3);
                            else
                                setSpeed('F', 3, 'B', 3);
                        }
                        if (rightSignal != 0){
                            /// received right signal
                            rightFlag = true;
                            if (rightSignal == 1)
                                setSpeed('B', 5, 'F', 3);
                            else if (rightSignal == 2)
                                setSpeed('B', 3, 'F', 5);
                            else
                                setSpeed('B', 3, 'F', 3);
                        }
                    }
                } else {
                    /// the robot collided,so we stop it
                    std::cout << "(auto_docking::pidControl) the robot is lost ?? " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - beginTime).count() << std::endl;
                    setSpeed('F', 0, 'F', 0);

                    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - beginTime).count() > 20){
                        /// TODO try again, send the robot back to the landing point or go backward ?
                        //Time interval of more than 20 seconds, check the battery status. The battery is not charging, the robot moves forward, re-looking for the correct location of the charging station
                        /*setSpeed( 'F', 10, 'F', 10);
                        time.sleep(10);
                        bumperCrashCount = 0;*/
                    }
                }

                return 0;
            } else
                std::cout << "(auto_docking::findChargingStation) could not call the bumpers service" << std::endl;
        } else
            std::cout << "(auto_docking::findChargingStation) could not call the short signal service" << std::endl;
    } else
        std::cout << "(auto_docking::findChargingStation) could not call the ir signal service" << std::endl;
    return -1;
}

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    std::cout << "(auto_docking::setSpeed) " << directionR << " " << velocityR << " " << directionL << " " << velocityL << std::endl;
    wheel::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("setSpeeds", speed);
}

/***************************************************************************************************/

void stopDocking(void){
    std::cout << "(auto_docking::stopDocking) called" << std::endl;
    docking = false;
    goalReached = false;

    /// unsubscribe so we don't receive messages for nothing
    goalStatusSubscriber.shutdown();
    robotPoseSubscriber.shutdown();

    /// if action server is up -> cancel
    if(ac->isServerConnected())
        ac->cancelAllGoals();
}

bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::cout << "(auto_docking::stopDockingService) service called" << std::endl;

    stopDocking();

    return true;
}

void initCurrentGoal(void){
    currentGoal.target_pose.header.frame_id = "map";
    currentGoal.target_pose.header.stamp = ros::Time::now();
    currentGoal.target_pose.pose.position.x = 0;
    currentGoal.target_pose.pose.position.y = 0;
    currentGoal.target_pose.pose.position.z = 0;
    currentGoal.target_pose.pose.orientation.x = 0;
    currentGoal.target_pose.pose.orientation.y = 0;
    currentGoal.target_pose.pose.orientation.z = 0;
    currentGoal.target_pose.pose.orientation.w = 1;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "auto_docking");
    ros::NodeHandle nh;

    ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));

    initCurrentGoal();

    ros::ServiceServer startDockingSrv = nh.advertiseService("startDocking", startDocking);
    ros::ServiceServer stopDockingSrv = nh.advertiseService("stopDocking", stopDockingService);

    ros::spin();
}