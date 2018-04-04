#include <gobot_base/twist.hpp>

#define CLIFF_THRESHOLD 170
#define CLIFF_OUTRANGE 1

bool collision = false, moved_from_collision = true, pause_robot = false;
double wheel_separation, wheel_radius, ticks_per_rotation, collision_threshold, avoid_spd;

bool bumper_on=false, cliff_on=false,moved_from_front_cliff = true,moved_from_back_cliff=true;
bool bumpers_broken[8]={false,false,false,false,false,false,false,false};

/// based on tests, the linear regression from the velocity in m/s to the ticks/sec is approx : y=15.606962627075x-2.2598795680051
//15.606962627075;//-2.2598795680051;
double a = 15.55, b = -2.26;  
double pi = 3.14159;

bool lost_robot = false;

bool enable_joy = false;
double linear_limit = 0.4, angular_limit = 0.8; 

int left_speed = 0, right_speed = 0;

ros::Time collisionTime;
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


void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff){
    if((cliff->cliff1>CLIFF_THRESHOLD) || (cliff->cliff1==CLIFF_OUTRANGE) || (cliff->cliff2>CLIFF_THRESHOLD) || (cliff->cliff2==CLIFF_OUTRANGE)){
        if(moved_from_front_cliff){
            //if robot is moving
            if(left_speed != 0 || right_speed != 0){
                if(!bumper_on){
                    moved_from_front_cliff = false;
                    SetRobot.setSound(3,1);
                    SetRobot.setMotorSpeed('B', avoid_spd, 'B', avoid_spd);
                }
            }
        }
    }
    else if(!moved_from_front_cliff){
        ros::Duration(0.5).sleep();
        moved_from_front_cliff=true;
        SetRobot.setMotorSpeed('F', 0, 'F', 0);
    }

    if((cliff->cliff3>CLIFF_THRESHOLD) || (cliff->cliff3==CLIFF_OUTRANGE) || (cliff->cliff4>CLIFF_THRESHOLD) || (cliff->cliff4==CLIFF_OUTRANGE)){
        if(moved_from_back_cliff){
            //if robot is moving
            if(left_speed != 0 || right_speed != 0){
                if(!bumper_on){
                    moved_from_back_cliff = false;
                    SetRobot.setSound(3,1);
                    SetRobot.setMotorSpeed('F', avoid_spd, 'F', avoid_spd);
                }
            }
        }
    }
    else if(!moved_from_back_cliff){
        ros::Duration(0.5).sleep();
        moved_from_back_cliff=true;
        SetRobot.setMotorSpeed('F', 0, 'F', 0);
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
        bumper_on = front || back;

        /// check if we have a collision
        if(front || back){
            /// if it's a new collision, we stop the robot
            if(!collision){
                SetRobot.setMotorSpeed('F', 0, 'F', 0);
                ROS_WARN("(twist::newBumpersInfo) just got a new collision");
                collision = true;
                collisionTime = ros::Time::now();
            } 
            else if((ros::Time::now() - collisionTime).toSec()>collision_threshold){
                bumper_collision_pub.publish(bumpers_data); 
                moved_from_collision = false; 
                /// if after collision_threshold seconds, the obstacle is still there, we go to the opposite direction
                //if front bumpers are trigered
                if(front && !back){
                    std::thread([](){
                        bumpers_broken[0] = !bumpers_data.bumper1;
                        bumpers_broken[1] = !bumpers_data.bumper2;
                        bumpers_broken[2] = !bumpers_data.bumper3;
                        bumpers_broken[3] = !bumpers_data.bumper4;
                        ROS_WARN("(twist::newBumpersInfo) Launching thread to move away from front obstacle");
                        SetRobot.setMotorSpeed('B', avoid_spd, 'B', avoid_spd);
                        ros::Duration(1.5).sleep();
                        SetRobot.setMotorSpeed('F', 0, 'F', 0);
                        //ROS_INFO("Front bumper broken: %d,%d,%d,%d",bumpers_broken[0],bumpers_broken[1],bumpers_broken[2],bumpers_broken[3]);
                        collisionTime = ros::Time::now();
                        moved_from_collision = true;
                    }).detach();
                } 
                //if back bumpers are trigered
                else if(back && !front){
                    std::thread([](){
                        bumpers_broken[4] = !bumpers_data.bumper5;
                        bumpers_broken[5] = !bumpers_data.bumper6;
                        bumpers_broken[6] = !bumpers_data.bumper7;
                        bumpers_broken[7] = !bumpers_data.bumper8;
                        ROS_WARN("(twist::newBumpersInfo) Launching thread to move away from back obstacle");
                        SetRobot.setMotorSpeed('F', avoid_spd, 'F', avoid_spd);
                        ros::Duration(1.5).sleep();
                        SetRobot.setMotorSpeed('F', 0, 'F', 0);
                        //ROS_INFO("Back bumper broken: %d,%d,%d,%d",bumpers_broken[4],bumpers_broken[5],bumpers_broken[6],bumpers_broken[7]);
                        collisionTime = ros::Time::now();
                        moved_from_collision = true;
                    }).detach();
                }
                //somehow, we are in this stage
                else{
                    collision = false;
                    moved_from_collision = true;
                    collisionTime = ros::Time::now();
                }
            }
        } 
        else if(collision){
            /// if we had a collision and the obstacle left
            ROS_INFO("(twist::newBumpersInfo) Obstacle left after %f seconds",(ros::Time::now() - collisionTime).toSec());
            ROS_INFO("Front bumper broken: %d,%d,%d,%d",bumpers_broken[0],bumpers_broken[1],bumpers_broken[2],bumpers_broken[3]);
            ROS_INFO("Back bumper broken: %d,%d,%d,%d",bumpers_broken[4],bumpers_broken[5],bumpers_broken[6],bumpers_broken[7]);
            collision = false;
            bumper_collision_pub.publish(bumpers_data);
        }
    }
}

void lostCallback(const std_msgs::Int8::ConstPtr& msg){
    if(msg->data==1 && !lost_robot){
        lost_robot=true;
        SetRobot.setLed(1,{"red","blue"});
    }
    else if(msg->data==0 && lost_robot){
        lost_robot=false;
        SetRobot.setLed(0,{"white"});
    }
}

void joyConnectionCallback(const std_msgs::Int8::ConstPtr& data){
    if(data->data == 0){
        if(enable_joy){
            enable_joy = false;
            SetRobot.setSound(3,1);
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
        }
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    //start -> enable manual control
    if(joy->buttons[7]){
        enable_joy = true;
        SetRobot.setMotorSpeed('F', 0, 'F', 0);
        SetRobot.setBatteryLed();
    }
    //back -> disable manual control
    if(joy->buttons[6]){
        enable_joy = false;
        SetRobot.setMotorSpeed('F', 0, 'F', 0);
        SetRobot.setBatteryLed();
    }
    if(enable_joy){
        //adjust linear speed
        if(joy->axes[7] == 1){
            linear_limit = (linear_limit+0.1) <= 0.9 ? linear_limit+0.1 : 0.9;
        }
        else if(joy->axes[7] == -1){
            linear_limit = (linear_limit-0.1) > 0.1 ? linear_limit-0.1 : 0.1;
        }

        //adjust angular speed
        if(joy->axes[6] == -1){
            angular_limit = (angular_limit+0.1) <= 2.0 ? angular_limit+0.1 : 2.0;
        }
        else if(joy->axes[6] == 1){
            angular_limit = (angular_limit-0.1) > 0.2 ? angular_limit-0.1 : 0.2;
        }

        //reset linear speed limit 0.4
        if(joy->buttons[9]){
            linear_limit = 0.4;
        }

        //reset angular speed limit 0.8
        if(joy->buttons[10]){
            angular_limit = 0.8;
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
            else {
                /// calculate the speed of each wheel in m/s
                double right_vel_m_per_sec = linear_limit * joy->axes[1] + angular_limit * joy->axes[3] * wheel_separation / (double)2;
                double left_vel_m_per_sec = linear_limit * joy->axes[1] - angular_limit * joy->axes[3] * wheel_separation / (double)2;

                /// calculate the real value we need to give the MD49
                double right_vel_speed = ((right_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;
                double left_vel_speed = ((left_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;

                SetRobot.setMotorSpeed(right_vel_speed >= 0 ? 'F' : 'B', fabs(right_vel_speed), left_vel_speed >= 0 ? 'F' : 'B', fabs(left_vel_speed));;
            }
        }
    }
}

void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist){
    if(enable_joy){
        if(robot_status_==5)
            ros::service::call("/gobot_command/pause_path",empty_srv);
        else if(robot_status_==15)
            ros::service::call("/gobot_command/stopGoDock",empty_srv);
        else if(robot_status_==25)
            ros::service::call("/gobot_command/stop_explore",empty_srv);
    }
    /// Received a new velocity cmd
    else if(!bumper_on && !cliff_on){
        /// if the velocity cmd is to tell the robot to stop, we just give 0 and don't calculate
        /// because with the linear regression function we would get a speed of ~0.002 m/s
        if(twist->linear.x == 0 && twist->angular.z == 0)
            SetRobot.setMotorSpeed('F', 0, 'F', 0);
        else {
            /// calculate the speed of each wheel in m/s
            double right_vel_m_per_sec = twist->linear.x + twist->angular.z * wheel_separation / 2.0;
            double left_vel_m_per_sec = twist->linear.x - twist->angular.z * wheel_separation / 2.0;

            /// calculate the real value we need to give the MD49
            double right_vel_speed = ((right_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;
            double left_vel_speed = ((left_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;

            //ROS_INFO("(twist::newCmdVel) new vel %f %f", right_vel_speed, left_vel_speed);

            SetRobot.setMotorSpeed(right_vel_speed >= 0 ? 'F' : 'B', fabs(right_vel_speed), left_vel_speed >= 0 ? 'F' : 'B', fabs(left_vel_speed));

            /// just to show the actual real speed we gave to the wheels
            //double real_vel = (a * (int)right_vel_speed + b) / ticks_per_rotation * 2 * pi * wheel_radius;
            //ROS_INFO("(twist::newCmdVel) linear vel %f m/s, compared to real vel %f  m/s\n so a difference of %f", twist->linear.x, real_vel, real_vel - twist->linear.x);
        }
    }
}

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
    left_speed = speed->velocityL;
    right_speed = speed->velocityR;
}

void statusCallback(const std_msgs::Int8::ConstPtr& msg){
    robot_status_ = msg->data;
}

void initParams(ros::NodeHandle &nh){
    nh.getParam("wheel_separation", wheel_separation);
    nh.getParam("wheel_radius", wheel_radius);
    nh.getParam("ticks_per_rotation", ticks_per_rotation);
    nh.getParam("collision_threshold", collision_threshold);
    nh.getParam("avoid_spd", avoid_spd);
}

void mySigintHandler(int sig){   
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist");
    ros::NodeHandle nh;
    SetRobot.initialize();
    signal(SIGINT, mySigintHandler);
    
    initParams(nh);
    
    ROS_INFO("(Twist) Waiting for MCU to be ready...");
    ros::service::waitForService("/gobot_startup/sensors_ready", ros::Duration(60.0));
    ROS_INFO("(Twist) MCU is ready.");

    bumper_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_topic", 1);
    bumper_collision_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_collision", 1);

    ros::Subscriber cmdVelSub = nh.subscribe("cmd_vel", 1, newCmdVel);
    ros::Subscriber bumpersSub = nh.subscribe("/gobot_base/bumpers_raw_topic", 1, newBumpersInfo);
    ros::Subscriber cliffSub = nh.subscribe("/gobot_base/cliff_topic", 1, cliffCallback);
    ros::Subscriber lostRobot = nh.subscribe("/gobot_recovery/lost_robot",1,lostCallback);
    ros::Subscriber joySub = nh.subscribe("joy", 1, joyCallback);
    ros::Subscriber joyConSub = nh.subscribe("joy_connection", 1, joyConnectionCallback);
    ros::Subscriber motorSpdSubscriber = nh.subscribe("/gobot_motor/motor_speed", 1, motorSpdCallback);
    ros::Subscriber statusSubscriber = nh.subscribe("/gobot_status/gobot_status", 1, statusCallback);

    //not in use now
    
    ros::ServiceServer continueRobot = nh.advertiseService("/gobot_base/continue_robot",continueRobotSrvCallback);
    ros::ServiceServer pauseRobot = nh.advertiseService("/gobot_base/pause_robot",pauseRobotSrvCallback);

    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}