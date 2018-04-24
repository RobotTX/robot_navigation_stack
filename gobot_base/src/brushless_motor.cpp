//ros headers
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <numeric> 
#include <serial/serial.h>
#include <thread>
#include <signal.h>

#include <gobot_msg_srv/robot_msgs.h>


bool testEncoder = false;
uint8_t leftSpeed_=128, rightSpeed_=128, rec_leftSpeed_=128,rec_rightSpeed_=128;

std::mutex serialMutex, dataMutex;
std::string MOTOR_PORT;
serial::Serial serialConnection;
int ROLLING_WINDOW_SIZE = 5;

//odom related data
int32_t last_left_encoder = 0, last_right_encoder = 0;
double x = 0.0, y = 0.0, th = 0.0, pi = 3.1415926;
int encoder_limit = 3000;
ros::Time current_time, last_time;

 //odom info
int ODOM_RATE=10;
double wheel_separation,wheel_radius,ticks_per_rotation;

/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead=0){
    std::vector<uint8_t> buff;
    
    /// Lock the mutex so no one can write at the same time
    serialMutex.lock();
    if(serialConnection.isOpen()){
        try{
            /// Send bytes to the MD49
            size_t bytes_wrote = serialConnection.write(toWrite);
            /// Read any byte that we are expecting
            if(bytesToRead > 0)
                serialConnection.read(buff, bytesToRead);

            //serialConnection.flush();
        } catch (std::exception& e) {
		    ROS_ERROR("MOTOR:: exception : %s", e.what());
	    }
    } 
    else {
        ROS_WARN("(MOTOR::writeAndRead) The serial connection is not opened, something is wrong");
    }

    /// Unlock the mutex
    serialMutex.unlock();

    return buff;
}

bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    dataMutex.lock();
    writeAndRead(std::vector<uint8_t>({0x37, 0x00,0x00}));
    last_left_encoder = 0;
    last_right_encoder = 0;
    x=0;
    y=0;
    th=0;
    last_time = current_time;
    dataMutex.unlock();
    return true;
}

bool brakeSrvCallback(gobot_msg_srv::SetInt::Request &req, gobot_msg_srv::SetInt::Response &res){
  //break
  if(req.data==1)
    writeAndRead(std::vector<uint8_t>({0x34, 0x01,0x01}));
  //release
  else
    writeAndRead(std::vector<uint8_t>({0x34, 0x00,0x00}));
}

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
    
    std::vector<uint8_t> spd_cmd(5,0x00);
    spd_cmd[0] = 0x33;
    spd_cmd[1] = speed->velocityL<200 ? speed->velocityL : 200;
    spd_cmd[2] = speed->directionL.compare("F")==0 ? 0x00 : 0x01;
    spd_cmd[3] = speed->velocityR<200 ? speed->velocityR : 200;
    spd_cmd[4] = speed->directionR.compare("F")==0 ? 0x00 : 0x01;
    writeAndRead(spd_cmd, 1);
}


bool initSerial(){
    serialMutex.lock();
    if(serialConnection.isOpen())
        serialConnection.close();

    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort("/dev/ttyUSB1");
    //Send 1200 bytes per second
    serialConnection.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serialConnection.setTimeout(timeout);
    serialConnection.close();
    serialConnection.open();

    //set speed to be 0
    serialConnection.write(std::vector<uint8_t>({0x33, 0x00, 0x00, 0x00, 0x00}));
    //release brake
    serialConnection.write(std::vector<uint8_t>({0x34, 0x00, 0x00}));
    
    serialMutex.unlock();
    return true;
}

void initParams(ros::NodeHandle &nh){
    nh.getParam("MOTOR_PORT", MOTOR_PORT);
    nh.getParam("WHEEL_SEP", wheel_separation);
    nh.getParam("WHEEL_RADIUS", wheel_radius);
    nh.getParam("TICKS_PER_ROT", ticks_per_rotation);
    nh.getParam("ODOM_RATE", ODOM_RATE);
    nh.getParam("ROLLING_WINDOW_SIZE", ROLLING_WINDOW_SIZE);
}

void mySigintHandler(int sig){   
    serialMutex.lock();
    try{
        //enable brake
        serialConnection.write(std::vector<uint8_t>({0x34, 0x01, 0x01}));
        //set speed to be 0
        serialConnection.write(std::vector<uint8_t>({0x33, 0x00, 0x00, 0x00, 0x00}));
        serialConnection.close();
	} catch (std::exception& e) {
		ROS_ERROR("MOTOR::Shutdown exception : %s", e.what());
	}
    serialMutex.unlock();
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "brushless_motor");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    initParams(nh);

    if(initSerial()){

        ros::ServiceServer resetOdomSrv = nh.advertiseService("/gobot_motor/resetOdom", resetOdom);
        ros::ServiceServer brakeSrv = nh.advertiseService("/gobot_motor/set_brake", brakeSrvCallback);

        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
        ros::Publisher odom_test_pub = nh.advertise<gobot_msg_srv::OdomTestMsg>("/odom_test", 1);
        ros::Publisher encoder_pub = nh.advertise<gobot_msg_srv::EncodersMsg>("/encoders", 1);
        ros::Publisher real_vel_pub = nh.advertise<geometry_msgs::Twist>("/real_vel", 1);
        
        ros::Subscriber motorSpd_sub = nh.subscribe("/gobot_motor/motor_speed", 1, motorSpdCallback);
        
        tf::TransformBroadcaster odom_broadcaster;
        last_time = ros::Time::now();

        ros::Rate r(1);
        while(ros::ok()){
            dataMutex.lock();
            std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x35,0x00,0x00}), 9);
            std::vector<uint8_t> rpms;
            //= writeAndRead(std::vector<uint8_t>({0x36,0x00,0x00}), 5);
            
            if(encoders.size()==9 && encoders.at(0)==0x95){
                int32_t leftEncoder = (encoders.at(1) << 24) + (encoders.at(2) << 16) + (encoders.at(3) << 8) + encoders.at(4);
                int32_t rightEncoder = (encoders.at(5) << 24) + (encoders.at(6) << 16) + (encoders.at(7) << 8) + encoders.at(8);
                current_time = ros::Time::now();
                double dt = (current_time - last_time).toSec();

                int32_t delta_left_encoder = leftEncoder - last_left_encoder;
                int32_t delta_right_encoder = rightEncoder - last_right_encoder;

                ROS_INFO("(MOTOR) right: %d, delta: %d; left: %d, delta: %d",rightEncoder,delta_right_encoder,leftEncoder,delta_left_encoder);
                last_left_encoder = leftEncoder;
                last_right_encoder = rightEncoder;

                // distance travelled by each wheel
                double left_dist = (delta_left_encoder / ticks_per_rotation) * 2.0 * wheel_radius * pi;
                double right_dist = (delta_right_encoder / ticks_per_rotation) * 2.0 * wheel_radius * pi;

                // velocity of each wheel
                double left_vel = left_dist / dt;
                double right_vel = right_dist / dt;
                //compute odometry in a typical way given the velocities of the robot
                double vel = (right_vel + left_vel) / 2.0;
                double vx = vel * cos(th);
                double vy = vel * sin(th);
                double vth = (right_vel - left_vel) / wheel_separation;
                ROS_INFO("(MOTOR) Linear vel:%.3f, Angular vel:%.3f",vel,vth);
                if(rpms.size()==5 && rpms.at(0)==0x95){
                  int16_t left_rpm = (rpms.at(1) << 8) + rpms.at(2);
                  int16_t right_rpm = (rpms.at(3) << 8) + rpms.at(4);
                  double l_rpm = left_rpm/100.0, r_rpm = right_rpm/100.0;
                  ROS_INFO("(MOTOR) left rpm: %.2f, right rpm: %.2f",l_rpm,r_rpm);
                }
                else{
                  ROS_INFO("(MOTOR) Skip rpm reading;");
                }

                double delta_x = vx * dt;
                double delta_y = vy * dt;
                double delta_th = vth * dt;

                x += delta_x;
                y += delta_y;
                th += delta_th;  
                //ROS_INFO("(MOTOR) %.5f,%.5f, %.5f",delta_x,delta_y,delta_th);
                
                //since all odometry is 6DOF we'll need a quaternion created from yaw
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

                //first, we'll publish the transform over tf
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";

                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;

                
                //send the transform
                odom_broadcaster.sendTransform(odom_trans);

                //next, we'll publish the odometry message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = ros::Time::now();;
                odom.header.frame_id = "odom";

                //set the position
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;

                //set the velocity
                odom.child_frame_id = "base_link";
                odom.twist.twist.linear.x = vel;
                odom.twist.twist.linear.y = 0.0;
                odom.twist.twist.angular.z = vth;

                //publish the message
                odom_pub.publish(odom);

                //publish encoders
                gobot_msg_srv::EncodersMsg encoder_msg;
                encoder_msg.left_encoder = leftEncoder;
                encoder_msg.right_encoder = rightEncoder;
                encoder_pub.publish(encoder_msg);


                last_time = current_time;
            }
            else
                ROS_WARN("(MOTOR::getEncoders) Got the wrong number of encoders data : %lu", encoders.size());
            
            dataMutex.unlock();
            r.sleep();
            ros::spinOnce();
        }
    } 
    else
        ROS_INFO("(MOTOR::main) Could not open the serial communication");

    return 0;
}
