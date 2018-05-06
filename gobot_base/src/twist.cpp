#include <gobot_base/twist.hpp>

#define PI 3.1415926

ros::Time collision_time;

bool pause_robot = false;
double WHEEL_SEP, WHEEL_RADIUS, TICKS_PER_ROT, WAIT_COLLISION, AVOID_SPD, FACTOP_VEL;

bool cliff_on=false,moved_from_front_cliff = true,moved_from_back_cliff=true;
double CLIFF_THRESHOLD = 170.0, CLIFF_OUTRANGE = 255.0;

bool bumper_on=false, collision = false, moved_from_collision = true;
bool bumpers_broken[8]={false,false,false,false,false,false,false,false};

/// based on tests, the linear regression from the velocity in m/s to the ticks/sec is approx : y=15.606962627075x-2.2598795680051
//15.606962627075;//-2.2598795680051;
double FACTOR_A = 15.6, FACTOR_B = -20.0;  

bool lost_robot = false;

bool joy_on = false;
double joy_linear_limit = 0.4, joy_angular_limit = 0.8, joy_linear = 0.4, joy_angular = 0.8; 

int left_speed_ = 0, right_speed_ = 0;

ros::Time bumper_collision_time;
ros::Publisher bumper_pub,bumper_collision_pub;

std_srvs::Empty empty_srv;
gobot_msg_srv::BumperMsg bumpers_data;

robot_class::SetRobot SetRobot;
int robot_status_ = -1;

bool continueRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    pause_robot=false;
    return true;
}

bool pauseRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    pause_robot=true;
    SetRobot.setMotorSpeed('F', 0, 'F', 0);
    return true;
}

bool joySpeedSrvCallback(gobot_msg_srv::SetFloatArray::Request &req, gobot_msg_srv::SetFloatArray::Response &res){
    joy_linear_limit = req.data[0];
    joy_angular_limit = req.data[1];
    joy_linear = joy_linear_limit;
    joy_angular = joy_angular_limit;
    return true;
}


bool cliffOutRange(double CliffData){
    if(CliffData>CLIFF_THRESHOLD || CliffData==CLIFF_OUTRANGE)
        return true;
    else
        return false;
}

void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff){
    //if robot is moving, check cliff sensors
    if(left_speed_ != 0 || right_speed_ != 0){
        if(cliffOutRange(cliff->cliff1) || cliffOutRange(cliff->cliff2)){
            if(moved_from_front_cliff && !bumper_on){
                moved_from_front_cliff = false;
                SetRobot.setMotorSpeed('B', AVOID_SPD, 'B', AVOID_SPD);
                SetRobot.setSound(3,1);
                SetRobot.setLed(1,{"red","white"});
                collision_time = ros::Time::now();
            }
        }

        else if(cliffOutRange(cliff->cliff3) || cliffOutRange(cliff->cliff4)){
            if(moved_from_back_cliff && !bumper_on){
                moved_from_back_cliff = false;
                SetRobot.setMotorSpeed('F', AVOID_SPD, 'F', AVOID_SPD);
                SetRobot.setSound(3,1);
                SetRobot.setLed(1,{"red","white"});
                collision_time = ros::Time::now();
            }
        }

        else if(!moved_from_back_cliff || !moved_from_front_cliff){
            ros::Duration(0.5).sleep();
            moved_from_front_cliff=true;
            moved_from_back_cliff=true;
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
            SetRobot.setSound(1,1);
        }
    }
    //if robot is not moving
    else if(cliff_on){
        moved_from_front_cliff = true;
        moved_from_back_cliff = true;
    }
    
    cliff_on = !(moved_from_front_cliff && moved_from_back_cliff);
}

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    /// 0 : collision; 1 : no collision
    if(bumpers_broken[0] && bumpers->bumper1)
        bumpers_broken[0] = false;
    if(bumpers_broken[1] && bumpers->bumper2)
        bumpers_broken[1] = false;
    if(bumpers_broken[2] && bumpers->bumper3)
        bumpers_broken[2] = false;
    if(bumpers_broken[3] && bumpers->bumper4)
        bumpers_broken[3] = false;
    if(bumpers_broken[4] && bumpers->bumper5)
        bumpers_broken[4] = false;
    if(bumpers_broken[5] && bumpers->bumper6)
        bumpers_broken[5] = false;
    if(bumpers_broken[6] && bumpers->bumper7)
        bumpers_broken[6] = false;
    if(bumpers_broken[7] && bumpers->bumper8)
        bumpers_broken[7] = false;

    bumpers_data.bumper1 = bumpers_broken[0] || bumpers->bumper1;
    bumpers_data.bumper2 = bumpers_broken[1] || bumpers->bumper2;
    bumpers_data.bumper3 = bumpers_broken[2] || bumpers->bumper3;
    bumpers_data.bumper4 = bumpers_broken[3] || bumpers->bumper4;
    bumpers_data.bumper5 = bumpers_broken[4] || bumpers->bumper5;
    bumpers_data.bumper6 = bumpers_broken[5] || bumpers->bumper6;
    bumpers_data.bumper7 = bumpers_broken[6] || bumpers->bumper7;
    bumpers_data.bumper8 = bumpers_broken[7] || bumpers->bumper8;

    //publish bumper data after detecting broken bumpers
    bumper_pub.publish(bumpers_data);

    if(moved_from_collision){
        bool front = !(bumpers_data.bumper1 && bumpers_data.bumper2 && bumpers_data.bumper3 && bumpers_data.bumper4);
        bool back = !(bumpers_data.bumper5 && bumpers_data.bumper6 && bumpers_data.bumper7 && bumpers_data.bumper8);
        /// check if we have a collision
        if(front || back){
            /// if it's a new collision, we stop the robot
            if(!collision){
                SetRobot.setMotorSpeed('F', 0, 'F', 0);
                ROS_WARN("(TWIST::newBumpersInfo) Detect collision");
                collision = true;
                bumper_collision_time = ros::Time::now();
                collision_time = ros::Time::now();
            } 
            else if((ros::Time::now()-bumper_collision_time) > ros::Duration(WAIT_COLLISION)){
                bumper_collision_pub.publish(bumpers_data); 
                moved_from_collision = false; 
                /// if after WAIT_COLLISION seconds, the obstacle is still there, we go to the opposite direction
                //if front bumpers are trigered
                if(front){
                    std::thread([](){
                        bumpers_broken[0] = !bumpers_data.bumper1;
                        bumpers_broken[1] = !bumpers_data.bumper2;
                        bumpers_broken[2] = !bumpers_data.bumper3;
                        bumpers_broken[3] = !bumpers_data.bumper4;
                        ROS_WARN("(TWIST::newBumpersInfo) Launching thread to move away from front obstacle");
                        SetRobot.setMotorSpeed('B', AVOID_SPD, 'B', AVOID_SPD);
                        ros::Duration(1.5).sleep();
                        SetRobot.setMotorSpeed('F', 0, 'F', 0);
                        //ROS_INFO("Front bumper broken: %d,%d,%d,%d",bumpers_broken[0],bumpers_broken[1],bumpers_broken[2],bumpers_broken[3]);
                        bumper_collision_time = ros::Time::now();
                        moved_from_collision = true;
                    }).detach();
                } 
                //if back bumpers are trigered
                else{
                    std::thread([](){
                        bumpers_broken[4] = !bumpers_data.bumper5;
                        bumpers_broken[5] = !bumpers_data.bumper6;
                        bumpers_broken[6] = !bumpers_data.bumper7;
                        bumpers_broken[7] = !bumpers_data.bumper8;
                        ROS_WARN("(TWIST::newBumpersInfo) Launching thread to move away from back obstacle");
                        SetRobot.setMotorSpeed('F', AVOID_SPD, 'F', AVOID_SPD);
                        ros::Duration(1.5).sleep();
                        SetRobot.setMotorSpeed('F', 0, 'F', 0);
                        //ROS_INFO("Back bumper broken: %d,%d,%d,%d",bumpers_broken[4],bumpers_broken[5],bumpers_broken[6],bumpers_broken[7]);
                        bumper_collision_time = ros::Time::now();
                        moved_from_collision = true;
                    }).detach();
                }
            }
        } 
        else if(collision){
            /// if we had a collision and the obstacle left
            ROS_INFO("(TWIST::newBumpersInfo) Obstacle left after %f seconds",(ros::Time::now() - bumper_collision_time).toSec());
            ROS_INFO("Front bumper broken: %d,%d,%d,%d",bumpers_broken[0],bumpers_broken[1],bumpers_broken[2],bumpers_broken[3]);
            ROS_INFO("Back bumper broken: %d,%d,%d,%d",bumpers_broken[4],bumpers_broken[5],bumpers_broken[6],bumpers_broken[7]);
            collision = false;
            bumper_collision_pub.publish(bumpers_data);
        }

        bumper_on = front || back;
    }
}

void lostCallback(const std_msgs::Int8::ConstPtr& msg){
    if(msg->data==1 && !lost_robot){
        lost_robot=true;
        //SetRobot.setLed(1,{"red","blue"});
    }
    else if(msg->data==0 && lost_robot){
        lost_robot=false;
        //SetRobot.setLed(0,{"white"});
    }
}

void joyConnectionCallback(const std_msgs::Int8::ConstPtr& data){
    if(data->data == 0){
        if(joy_on){
            joy_on = false;
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
        }
    }
    SetRobot.setSound(2,1);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    //start -> enable manual control
    if(joy->buttons[7]){
        joy_on = true;
        SetRobot.setMotorSpeed('F', 0, 'F', 0);
        SetRobot.setBatteryLed();
    }
    //back -> disable manual control
    if(joy->buttons[6]){
        joy_on = false;
        SetRobot.setMotorSpeed('F', 0, 'F', 0);
        SetRobot.setBatteryLed();
    }
    if(joy_on){
        //adjust linear speed
        if(joy->axes[7] == 1){
            joy_linear = (joy_linear+0.1) <= 0.9 ? joy_linear+0.1 : 0.9;
        }
        else if(joy->axes[7] == -1){
            joy_linear = (joy_linear-0.1) > 0.1 ? joy_linear-0.1 : 0.1;
        }

        //adjust angular speed
        if(joy->axes[6] == -1){
            joy_angular = (joy_angular+0.1) <= 2.0 ? joy_angular+0.1 : 2.0;
        }
        else if(joy->axes[6] == 1){
            joy_angular = (joy_angular-0.1) > 0.2 ? joy_angular-0.1 : 0.2;
        }

        //reset linear speed limit
        if(joy->buttons[9]){
            joy_linear = joy_linear_limit;
        }

        //reset angular speed limit
        if(joy->buttons[10]){
            joy_angular = joy_angular_limit;
        }

        if(joy->buttons[0])
            SetRobot.setLed(0,{"green"});
        if(joy->buttons[1])
            SetRobot.setLed(0,{"red"});
        if(joy->buttons[2])
            SetRobot.setLed(0,{"blue"});
        if(joy->buttons[3])
            SetRobot.setLed(0,{"yellow"});

        if(!bumper_on && !cliff_on){
            if(joy->axes[1] == 0 && joy->axes[3] == 0)
                SetRobot.setMotorSpeed('F', 0, 'F', 0);
            else
                cmdToMotorSpeed(joy_linear*joy->axes[1], joy_angular*joy->axes[3]);
        }
    }
    else{
        //LT || RT
        if(joy->axes[2]==-1 || joy->axes[5]==-1){
            //button A
            if(joy->buttons[0])
                ros::service::call("/gobot_command/play_path",empty_srv);
            //button B
            else if(joy->buttons[1])
                ros::service::call("/gobot_command/stop_path",empty_srv);
            //button X
            else if(joy->buttons[2])
                ros::service::call("/gobot_command/pause_path",empty_srv);
            //button Y
            else if(joy->buttons[3])
                ros::service::call("/gobot_command/goDock",empty_srv);
        }
    }
}

void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist){
    if(joy_on){
        int move_status = SetRobot.stopRobotMoving();
        if(move_status != 0){
            ROS_INFO("(TWIST) Stop robot motion, current status: %d.", move_status);
        }
    }
    /// Received a new velocity cmd
    else if(!bumper_on && !cliff_on){
        if(twist->linear.x == 0 && twist->angular.z == 0)
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
        else
            cmdToMotorSpeed(twist->linear.x, twist->angular.z);
    }
}

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
    left_speed_ = speed->velocityL;
    right_speed_ = speed->velocityR;
}

void statusCallback(const std_msgs::Int8::ConstPtr& msg){
    robot_status_ = msg->data;
}

void cmdToMotorSpeed(double cmd_linear, double cmd_angular){
    /// calculate the speed of each wheel in m/s
    double right_vel_m_per_sec = cmd_linear + cmd_angular * WHEEL_SEP / 2.0;
    double left_vel_m_per_sec = cmd_linear - cmd_angular * WHEEL_SEP / 2.0;

    /// calculate the real value we need to give the MD49
    double right_vel_speed = (right_vel_m_per_sec * FACTOP_VEL - FACTOR_B ) / FACTOR_A;
    double left_vel_speed = (left_vel_m_per_sec * FACTOP_VEL - FACTOR_B ) / FACTOR_A;

    //ROS_INFO("(TWIST::newCmdVel) new vel %f %f", right_vel_speed, left_vel_speed);
    SetRobot.setMotorSpeed(right_vel_speed >= 0 ? 'F' : 'B', fabs(right_vel_speed), left_vel_speed >= 0 ? 'F' : 'B', fabs(left_vel_speed));
}

void initParams(ros::NodeHandle &nh){
    nh.getParam("FACTOR_A", FACTOR_A);
    nh.getParam("FACTOR_B", FACTOR_B);
    nh.getParam("AVOID_SPD", AVOID_SPD);
    nh.getParam("WHEEL_SEP", WHEEL_SEP);
    nh.getParam("WHEEL_RADIUS", WHEEL_RADIUS);
    nh.getParam("TICKS_PER_ROT", TICKS_PER_ROT);
    nh.getParam("WAIT_COLLISION", WAIT_COLLISION);
    nh.getParam("CLIFF_OUTRANGE", CLIFF_OUTRANGE);
    nh.getParam("CLIFF_THRESHOLD", CLIFF_THRESHOLD);

    FACTOP_VEL = TICKS_PER_ROT/(2*PI*WHEEL_RADIUS);
}

void mySigintHandler(int sig){   
    ros::shutdown();
}

//if robot stays in collision status too long, reset the collision flag
void checkCollisionTimer(const ros::TimerEvent&){
    if((ros::Time::now()-collision_time) > ros::Duration(30.0)){
        if(cliff_on || bumper_on){
            cliff_on = false;
            bumper_on = false;
            collision_time = ros::Time::now();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    SetRobot.initialize();
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(TWIST::START) Waiting for SENSORS to be ready...");
    ros::service::waitForService("/gobot_startup/sensors_ready", ros::Duration(60.0));
    ROS_INFO("(TWIST::START) SENSORS is ready.");
    //Startup end
    
    initParams(nh);

    bumper_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_topic", 1);
    bumper_collision_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_collision", 1);

    ros::Subscriber cmdVel_sub = nh.subscribe("cmd_vel", 1, newCmdVel);
    ros::Subscriber bumpers_sub = nh.subscribe("/gobot_base/bumpers_raw_topic", 1, newBumpersInfo);
    ros::Subscriber cliff_sub = nh.subscribe("/gobot_base/cliff_topic", 1, cliffCallback);
    ros::Subscriber lostRobot_sub = nh.subscribe("/gobot_recovery/lost_robot",1,lostCallback);
    ros::Subscriber joy_sub = nh.subscribe("joy", 1, joyCallback);
    ros::Subscriber joyConnection_sub = nh.subscribe("joy_connection", 1, joyConnectionCallback);
    ros::Subscriber motorSpd_sub = nh.subscribe("/gobot_motor/motor_speed", 1, motorSpdCallback);
    ros::Subscriber status_sub = nh.subscribe("/gobot_status/gobot_status", 1, statusCallback);

    ros::ServiceServer joySpeed = nh.advertiseService("/gobot_base/set_joy_speed",joySpeedSrvCallback);

    //not in use now
    ros::ServiceServer continueRobot = nh.advertiseService("/gobot_base/continue_robot",continueRobotSrvCallback);
    ros::ServiceServer pauseRobot = nh.advertiseService("/gobot_base/pause_robot",pauseRobotSrvCallback);

    collision_time = ros::Time::now();
    ros::Timer collision_timer = nh.createTimer(ros::Duration(20), checkCollisionTimer);

    ros::spin();
    return 0;
}