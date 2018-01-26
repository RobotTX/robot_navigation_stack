#include <gobot_base/twist.hpp>

#define CLIFF_THRESHOLD 170
#define CLIFF_OUTRANGE 0

std::shared_ptr<MoveBaseClient> ac(0);
std_srvs::Empty empty_srv;
bool collision = false;
bool moved_from_collision = true;
bool pause_robot = false;
double wheel_separation, wheel_radius, ticks_per_rotation, collision_threshold, avoid_spd;

ros::Time collisionTime;

ros::Publisher bumper_pub,bumper_collision_pub;

bool bumper_on=false, cliff_on=false,moved_from_front_cliff = true,moved_from_back_cliff=true;
bool bumpers_broken[8]={false,false,false,false,false,false,false,false};
gobot_msg_srv::BumperMsg bumpers_data;

/// based on tests, the linear regression from the velocity in m/s to the ticks/sec is approx : y=15.606962627075x-2.2598795680051
//15.606962627075;//-2.2598795680051;
double a = 15.55, b = -2.26;  

double pi = 3.14159;

bool enable_joy = false;
double linear_limit = 0.4, angular_limit = 0.8; 

robot_class::SetRobot robot;

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    //ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR , velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("/gobot_motor/setSpeeds", speed);
}

bool continueRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    pause_robot=false;
    return true;
}

bool pauseRobotSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    pause_robot=true;
    setSpeed('F', 0, 'F', 0);
    return true;
}


void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliff){
    if((cliff->cliff1>CLIFF_THRESHOLD) || (cliff->cliff1==CLIFF_OUTRANGE) || (cliff->cliff2>CLIFF_THRESHOLD) || (cliff->cliff2==CLIFF_OUTRANGE)){
        if(moved_from_front_cliff){
            gobot_msg_srv::GetInt get_speed;
            ros::service::call("/gobot_motor/getSpeeds",get_speed);
            //if robot is moving
            if(get_speed.response.data[0]!=128 || get_speed.response.data[1]!=128){
                if(!bumper_on){
                    moved_from_front_cliff = false;
                    robot.setSound(3,1);
                    setSpeed('B', avoid_spd, 'B', avoid_spd);
                }
            }
        }
    }
    else if(!moved_from_front_cliff){
        ros::Duration(0.5).sleep();
        moved_from_front_cliff=true;
        setSpeed('F', 0, 'F', 0);
    }

    if((cliff->cliff3>CLIFF_THRESHOLD) || (cliff->cliff3==CLIFF_OUTRANGE) || (cliff->cliff4>CLIFF_THRESHOLD) || (cliff->cliff4==CLIFF_OUTRANGE)){
        if(moved_from_back_cliff){
            gobot_msg_srv::GetInt get_speed;
            ros::service::call("/gobot_motor/getSpeeds",get_speed);
            //if robot is moving
            if(get_speed.response.data[0]!=128 || get_speed.response.data[1]!=128)
                if(!bumper_on){
                    moved_from_back_cliff = false;
                    robot.setSound(3,1);
                    setSpeed('F', avoid_spd, 'F', avoid_spd);
                }
        }
    }
    else if(!moved_from_back_cliff){
        ros::Duration(0.5).sleep();
        moved_from_back_cliff=true;
        setSpeed('F', 0, 'F', 0);
    }
    cliff_on = !(moved_from_front_cliff && moved_from_back_cliff);
}

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    /// 0 : collision; 1 : no collision
    bumpers_data.bumper1 = bumpers_broken[0] || bumpers->bumper1;
    bumpers_data.bumper2 = bumpers_broken[1] || bumpers->bumper2;
    bumpers_data.bumper3 = bumpers_broken[2] || bumpers->bumper3;
    bumpers_data.bumper4 = bumpers_broken[3] || bumpers->bumper4;
    bumpers_data.bumper5 = bumpers_broken[4] || bumpers->bumper5;
    bumpers_data.bumper6 = bumpers_broken[5] || bumpers->bumper6;
    bumpers_data.bumper7 = bumpers_broken[6] || bumpers->bumper7;
    bumpers_data.bumper8 = bumpers_broken[7] || bumpers->bumper8;

    bool front = !(bumpers_data.bumper1 && bumpers_data.bumper2 && bumpers_data.bumper3 && bumpers_data.bumper4);
    bool back = !(bumpers_data.bumper5 && bumpers_data.bumper6 && bumpers_data.bumper7 && bumpers_data.bumper8);
    bumper_on = front || back;

    if(moved_from_collision){
        bool broken = bumpers_broken[0]||bumpers_broken[1]||bumpers_broken[2]||bumpers_broken[3]
        ||bumpers_broken[4]||bumpers_broken[5]||bumpers_broken[6]||bumpers_broken[7]; 

        if(broken && !bumper_on){
            for(int i=0;i<sizeof(bumpers_broken);i++)
                bumpers_broken[i]=false; 
        }
    }

    //publish bumper data after detecting broken bumpers
    bumper_pub.publish(bumpers_data);

    /// check if we have a collision
    if(bumper_on){
        /// if it's a new collision, we stop the robot
        if(!collision){
            ROS_WARN("(twist::newBumpersInfo) just got a new collision");
            collision = true;
            collisionTime = ros::Time::now();
            setSpeed('F', 0, 'F', 0);
        } 
        else if((ros::Time::now() - collisionTime).toSec()>collision_threshold && moved_from_collision){
        /// if after 5 seconds, the obstacle is still there, we go to the opposite direction
            moved_from_collision = false;
            bumper_collision_pub.publish(bumpers_data);
            /// We create a thread that will make the robot go into the opposite direction, then sleep for 2 seconds and make the robot stop
            if(front && !back){  
                std::thread([](){
                    ROS_WARN("(twist::newBumpersInfo) Launching thread to move away from front obstacle");
                    setSpeed('B', avoid_spd, 'B', avoid_spd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                    setSpeed('F', 0, 'F', 0);
                    bumpers_broken[0] = !bumpers_data.bumper1;
                    bumpers_broken[1] = !bumpers_data.bumper2;
                    bumpers_broken[2] = !bumpers_data.bumper3;
                    bumpers_broken[3] = !bumpers_data.bumper4;
                    if(bumpers_broken[0] || bumpers_broken[1] || bumpers_broken[2] || bumpers_broken[3])
                        ROS_INFO("Front bumper broken: %d,%d,%d,%d",bumpers_broken[0],bumpers_broken[1],bumpers_broken[2],bumpers_broken[3]);
                    collisionTime = ros::Time::now();
                    moved_from_collision = true;
                }).detach();
            } 
            else if(back && !front){
                std::thread([](){
                    ROS_WARN("(twist::newBumpersInfo) Launching thread to move away from back obstacle");
                    setSpeed('F', avoid_spd, 'F', avoid_spd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                    setSpeed('F', 0, 'F', 0);
                    bumpers_broken[4] = !bumpers_data.bumper5;
                    bumpers_broken[5] = !bumpers_data.bumper6;
                    bumpers_broken[6] = !bumpers_data.bumper7;
                    bumpers_broken[7] = !bumpers_data.bumper8;
                    if(bumpers_broken[4] || bumpers_broken[5] || bumpers_broken[6] || bumpers_broken[7])
                        ROS_INFO("Back bumper broken: %d,%d,%d,%d",bumpers_broken[4],bumpers_broken[5],bumpers_broken[6],bumpers_broken[7]);
                    collisionTime = ros::Time::now();
                    moved_from_collision = true;
                }).detach();
            }
        }
    } 
    else if(collision && moved_from_collision){
        /// if we had a collision and the obstacle left
        bumper_collision_pub.publish(bumpers_data);
        ROS_INFO("(twist::newBumpersInfo) Obstacle left after %f seconds",(ros::Time::now() - collisionTime).toSec());
        collision = false;
    }
}

void joyConnectionCallback(const std_msgs::Int8::ConstPtr& data){
    if(data->data == 0){
        if(enable_joy){
            enable_joy = false;
            robot.setSound(3,1);
            setSpeed('F', 0, 'F', 0);
        }
    }
    else if(data->data == 1){
        if(!enable_joy){
            enable_joy = true;
            robot.setSound(2,1);
            linear_limit = 0.4;
            angular_limit = 0.8;
            setSpeed('F', 0, 'F', 0);
        }
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    //start -> enable manual control
    if(joy->buttons[7]){
        enable_joy = true;
        robot.setBatteryLed();
    }
    //back -> disable manual control
    if(joy->buttons[6]){
        enable_joy = false;
        setSpeed('F', 0, 'F', 0);
        robot.setBatteryLed();
    }
    if(enable_joy){
        //adjust linear speed
        if(joy->axes[7] == 1){
            linear_limit = (linear_limit+0.1) <= 0.9 ? linear_limit+0.1 : 0.9;
        }
        else if(joy->axes[7] == -1){
            linear_limit = (linear_limit-0.1) > 0 ? linear_limit-0.1 : 0.1;
        }

        //adjust angular speed
        if(joy->axes[6] == -1){
            angular_limit = (angular_limit+0.1) <= 2.0 ? angular_limit+0.1 : 2.0;
        }
        else if(joy->axes[6] == 1){
            angular_limit = (angular_limit-0.1) > 0 ? angular_limit-0.1 : 0.1;
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
            robot.setPermanentLed("green");
        if(joy->buttons[1])
            robot.setPermanentLed("red");
        if(joy->buttons[2])
            robot.setPermanentLed("blue");
        if(joy->buttons[3])
            robot.setPermanentLed("yellow");

        if(!collision && !cliff_on){
            if(joy->axes[1] == 0 && joy->axes[3] == 0)
                setSpeed('F', 0, 'F', 0);
            else {
                /// calculate the speed of each wheel in m/s
                double right_vel_m_per_sec = linear_limit * joy->axes[1] + angular_limit * joy->axes[3] * wheel_separation / (double)2;
                double left_vel_m_per_sec = linear_limit * joy->axes[1] - angular_limit * joy->axes[3] * wheel_separation / (double)2;

                /// calculate the real value we need to give the MD49
                double right_vel_speed = ((right_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;
                double left_vel_speed = ((left_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;

                setSpeed(right_vel_speed >= 0 ? 'F' : 'B', fabs(right_vel_speed), left_vel_speed >= 0 ? 'F' : 'B', fabs(left_vel_speed));;
            }
        }
    }
}

void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist){
    if(enable_joy){
        if(ac->isServerConnected())
            ac->cancelAllGoals();
    }
    /// Received a new velocity cmd
    else if(!collision && !pause_robot && !cliff_on){
        /// if the velocity cmd is to tell the robot to stop, we just give 0 and don't calculate
        /// because with the linear regression function we would get a speed of ~0.002 m/s
        if(twist->linear.x == 0 && twist->angular.z == 0)
            setSpeed('F', 0, 'F', 0);
        else {
            /// calculate the speed of each wheel in m/s
            double right_vel_m_per_sec = twist->linear.x + twist->angular.z * wheel_separation / (double)2;
            double left_vel_m_per_sec = twist->linear.x - twist->angular.z * wheel_separation / (double)2;

            /// calculate the real value we need to give the MD49
            double right_vel_speed = ((right_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;
            double left_vel_speed = ((left_vel_m_per_sec * ticks_per_rotation) / (2 * pi * wheel_radius) - b ) / a;

            //ROS_INFO("(twist::newCmdVel) new vel %f %f", right_vel_speed, left_vel_speed);

            setSpeed(right_vel_speed >= 0 ? 'F' : 'B', fabs(right_vel_speed), left_vel_speed >= 0 ? 'F' : 'B', fabs(left_vel_speed));

            /// just to show the actual real speed we gave to the wheels
            //double real_vel = (a * (int)right_vel_speed + b) / ticks_per_rotation * 2 * pi * wheel_radius;
            //ROS_INFO("(twist::newCmdVel) linear vel %f m/s, compared to real vel %f  m/s\n so a difference of %f", twist->linear.x, real_vel, real_vel - twist->linear.x);
        }
    }
}

bool initParams(){
    ros::NodeHandle nh;

    nh.getParam("wheel_separation", wheel_separation);
    nh.getParam("wheel_radius", wheel_radius);
    nh.getParam("ticks_per_rotation", ticks_per_rotation);
    nh.getParam("collision_threshold", collision_threshold);
    nh.getParam("avoid_spd", avoid_spd);

    std::cout << "(twist::initParams) wheel_separation:"<<wheel_separation<<" wheel_radius:"<<wheel_radius
    <<" ticks_per_rotation:"<<ticks_per_rotation<<" collision_threshold:"<<collision_threshold
    <<" avoid_spd:"<<avoid_spd<<std::endl;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist");
    ros::NodeHandle nh;

    if(initParams()){
        bumper_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_topic", 1);
        bumper_collision_pub = nh.advertise<gobot_msg_srv::BumperMsg>("/gobot_base/bumpers_collision", 1);

        ros::Subscriber cmdVelSub = nh.subscribe("cmd_vel", 1, newCmdVel);
        ros::Subscriber bumpersSub = nh.subscribe("/gobot_base/bumpers_raw_topic", 1, newBumpersInfo);
        ros::Subscriber cliffSub = nh.subscribe("/gobot_base/cliff_topic", 1, cliffCallback);
        ros::Subscriber joySub = nh.subscribe("joy", 1, joyCallback);
        ros::Subscriber joyConSub = nh.subscribe("joy_connection", 1, joyConnectionCallback);

        //not in use now
        ros::ServiceServer continueRobot = nh.advertiseService("/gobot_base/continue_robot",continueRobotSrvCallback);
        ros::ServiceServer pauseRobot = nh.advertiseService("/gobot_base/pause_robot",pauseRobotSrvCallback);

        ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));

        ros::spin();
    }

    return 0;
}