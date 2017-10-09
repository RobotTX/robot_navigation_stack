#include <gobot_base/twist.hpp>

bool collision = false;
bool moving_from_collision = false;
bool moved_away_from_collision = false;
bool pause_robot = false;

std::chrono::system_clock::time_point collisionTime;


bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    //ROS_INFO("(auto_docking::setSpeed) %c %d %c %d", directionR , velocityR, directionL, velocityL);
    gobot_msg_srv::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("setSpeeds", speed);
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

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){

    /// 0 : collision; 1 : no collision
    bool front = !(bumpers->bumper1 && bumpers->bumper2 && bumpers->bumper3 && bumpers->bumper4);
    bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);

    //ROS_INFO("(twist::newBumpersInfo) Bumpers: %d %d %d %d || %d %d %d %d ", bumpers->bumper1, bumpers->bumper2, bumpers->bumper3, bumpers->bumper4, bumpers->bumper5, bumpers->bumper6, bumpers->bumper7, bumpers->bumper8);

    /// check if we have a collision
    if(front || back){
        /// if it's a new collision, we stop the robot
        if(!collision){
            ROS_WARN("(twist::newBumpersInfo) just got a new collision");
            collision = true;
            collisionTime = std::chrono::system_clock::now();
            setSpeed('F', 0, 'F', 0);
        } else {
            if(moving_from_collision){
                ROS_WARN("(twist::newBumpersInfo) just got a new collision");
                collisionTime = std::chrono::system_clock::now();
                setSpeed('F', 0, 'F', 0);
                moved_away_from_collision = false;
            }
            /// if after 6 seconds, the obstacle is still there, we go to the opposite direction
            if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - collisionTime).count() > 6 
                && !moved_away_from_collision){
                moving_from_collision = true;
                moved_away_from_collision = true;
                /// We create a thread that will make the robot go into the opposite direction, then sleep for 2 seconds and make the robot stop
                if(front && !back){
                    std::thread([](){
                        ROS_WARN("(twist::newBumpersInfo) Launching thread to move away from the obstacle");
                        setSpeed('B', 5, 'B', 5);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        setSpeed('F', 0, 'F', 0);
                        moving_from_collision = false;
                    }).detach();
                } else if(back && !front){
                    std::thread([](){
                        ROS_WARN("(twist::newBumpersInfo) Launching thread to move away from the obstacle");
                        setSpeed('F', 5, 'F', 5);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        setSpeed('F', 0, 'F', 0);
                        moving_from_collision = false;
                    }).detach();
                }
            }
            /// TODO check if the obstacle is still there after moving away from it
            /// probably means we got a bumper problem => send a message to the user  
        }
    } else {
        /// if we had a collision and the obstacle left
        if(collision && !moving_from_collision){
            ROS_INFO("(twist::newBumpersInfo) the obstacle left after %f seconds", (double) (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - collisionTime).count() / 1000));
            collision = false;
            moved_away_from_collision = false;
        }
    }
}

void newCmdVel(const geometry_msgs::Twist::ConstPtr& twist){
    /// Received a new velocity cmd
    if(!collision && !pause_robot){
        ros::NodeHandle nh;
        double wheel_separation;
        nh.getParam("wheel_separation", wheel_separation);
        double wheel_radius;
        nh.getParam("wheel_radius", wheel_radius);
        double ticks_per_rotation;
        nh.getParam("ticks_per_rotation", ticks_per_rotation);

        /// if the velocity cmd is to tell the robot to stop, we just give 0 and don't calculate
        /// because with the linear regression function we would get a speed of ~0.002 m/s
        if(twist->linear.x == 0 && twist->angular.z == 0)
            setSpeed('F', 0, 'F', 0);
        else {
            /// calculate the speed of each wheel in m/s
            double right_vel_m_per_sec = twist->linear.x + twist->angular.z * wheel_separation / (double)2;
            double left_vel_m_per_sec = twist->linear.x - twist->angular.z * wheel_separation / (double)2;

            /// based on tests, the linear regression from the velocity in m/s to the ticks/sec is approx : y=15.606962627075x-2.2598795680051
            //tx//m/s->ticks/sec: vmd=((v*Nticks)/(2*pi*r)-b)/a
            double a = 15.606962627075;
            double b = -2.2598795680051;

            /// calculate the real value we need to give the MD49
            double right_vel_speed = ((right_vel_m_per_sec * ticks_per_rotation) / (2 * 3.14159 * wheel_radius) - b ) / a;
            double left_vel_speed = ((left_vel_m_per_sec * ticks_per_rotation) / (2 * 3.14159 * wheel_radius) - b ) / a;

            //ROS_INFO("(twist::newCmdVel) new vel %f %f", right_vel_speed, left_vel_speed);

            setSpeed(right_vel_speed >= 0 ? 'F' : 'B', abs(right_vel_speed), left_vel_speed >= 0 ? 'F' : 'B', abs(left_vel_speed));

            /// just to show the actual real speed we gave to the wheels
            //double real_vel = (a * (int)right_vel_speed + b) / ticks_per_rotation * 2 * 3.14159 * wheel_radius;
            //ROS_INFO("(twist::newCmdVel) linear vel %f m/s, compared to real vel %f  m/s\n so a difference of %f", twist->linear.x, real_vel, real_vel - twist->linear.x);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twist");

    ros::NodeHandle nh;

    ros::Subscriber bumpersSub = nh.subscribe("bumpers_topic", 1, newBumpersInfo);
    ros::Subscriber cmdVelSub = nh.subscribe("cmd_vel", 1, newCmdVel);
    ros::ServiceServer continueRobot = nh.advertiseService("/gobot_base/continue_robot",continueRobotSrvCallback);
    ros::ServiceServer pauseRobot = nh.advertiseService("/gobot_base/pause_robot",pauseRobotSrvCallback);

    ros::spin();

    return 0;
}