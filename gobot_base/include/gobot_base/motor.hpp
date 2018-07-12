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

#include <gobot_msg_srv/robot_msgs.h>


#define PI_ 3.1415926

class MotorClass {
    public:
        MotorClass(): 
        test_encoders_(false),reset_odom_(false),reset_encoder_(false),encoder_limit_(3000),leftSpeed_(128),rightSpeed_(128), rec_leftSpeed_(128),rec_rightSpeed_(128),
        last_left_encoder_(0), last_right_encoder_(0), odom_x_(0), odom_y_(0), odom_th_(0)
        { 
            ros::NodeHandle nh;
            //load parameters
            nh.getParam("MOTOR_PORT", motor_device_);
            nh.getParam("WHEEL_SEP", wheel_sep_);
            nh.getParam("WHEEL_RADIUS", wheel_radius_);
            nh.getParam("TICKS_PER_ROT", ticks_per_rot_);
            nh.getParam("ODOM_RATE", odom_rate_);
            
            if(!initSerial()){
                exit(1);
            }

            resetOdomSrv = nh.advertiseService("/gobot_motor/reset_odom", &MotorClass::resetOdom, this);
            resetEncodersSrv = nh.advertiseService("/gobot_motor/reset_encoders", &MotorClass::resetEncoders, this);

            odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
            real_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/real_vel", 1);
            encoder_pub_ = nh.advertise<gobot_msg_srv::EncodersMsg>("/encoders", 1);
            odom_test_pub_ = nh.advertise<gobot_msg_srv::OdomTestMsg>("/odom_test", 1);
            
            motorSpd_sub_ = nh.subscribe("/gobot_motor/motor_speed", 1, &MotorClass::motorSpdCallback, this);

            //set up the planner's thread
            odom_thread_ = new boost::thread(&MotorClass::odomThread, this);


            //Startup begin
            motorReadySrv = nh.advertiseService("/gobot_startup/motor_ready", &MotorClass::motorReadySrvCallback,this);
            //Startup end
        }

        ~MotorClass(){
            motorSpd_sub_.shutdown();

            odom_thread_->interrupt();
            odom_thread_->join();
            delete odom_thread_;

            serialMutex_.lock();
            try{
                //set speed to 0 when shutdown
                serialConnection.write(std::vector<uint8_t>({0x00, 0x31, 0x80, 0x00, 0x32, 0x80}));
                serialConnection.flush();
                serialConnection.close();
            } catch (std::exception& e) {
                std::cout<<"(MOTOR::Shutdown) exception : "<<e.what()<<std::endl;
            }
            serialMutex_.unlock();
        }

        /// Initialize the serial connection
        bool initSerial(){
            /// Get the port in which our device is connected
            ROS_INFO("(MOTOR::initSerial) MD49 port : %s", motor_device_.c_str());

            serialMutex_.lock();
            if(serialConnection.isOpen())
                serialConnection.close();

            // Set the serial port, baudrate and timeout in milliseconds
            serialConnection.setPort(motor_device_);
            //Send 1200 bytes per second
            serialConnection.setBaudrate(9600);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
            serialConnection.setTimeout(timeout);
            serialConnection.close();
            serialConnection.open();


            if(serialConnection.isOpen()){
                /// First 3 bytes : set the mode (see MD49 documentation)
                /// then 2 bytes to disable the 2 sec timeout
                /// then 3 bytes to set the acceleration steps (1-10)
                serialConnection.write(std::vector<uint8_t>({0x00, 0x34, 0x00,0x00, 0x38,0x00, 0x33, 0x01}));
                //set speed to 0 when initialize
                serialConnection.write(std::vector<uint8_t>({0x00, 0x31, 0x80, 0x00, 0x32, 0x80}));
                ///reset Encoder
                serialConnection.write(std::vector<uint8_t>({0x00, 0x35}));
                ROS_INFO("(MOTOR::initSerial) Established connection to motor.");
                serialMutex_.unlock();
                return true;
            } 
            else{
                serialMutex_.unlock();
                return false;
            }
        }


        void odomThread(){
            last_time = ros::Time::now();
            ros::Rate r(odom_rate_);

            while(ros::ok()){
                if(reset_encoder_){
                    reset_encoder_ = false;
                    last_left_encoder_ = 0;
                    last_right_encoder_ = 0;
                    writeAndRead(std::vector<uint8_t>({0x00, 0x35}));
                }
                std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);
                if(encoders.size() == 8){
                    int32_t leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
                    int32_t rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
                    
                    //publish encoders
                    gobot_msg_srv::EncodersMsg encoder_msg;
                    encoder_msg.left_encoder = leftEncoder;
                    encoder_msg.right_encoder = rightEncoder;
                    encoder_pub_.publish(encoder_msg);

                    current_time = ros::Time::now();
                    double dt = (current_time - last_time).toSec();

                    int32_t delta_left_encoder = leftEncoder - last_left_encoder_;
                    int32_t delta_right_encoder = rightEncoder - last_right_encoder_;

                    // difference of ticks compared to last time
                    //122rpm, 2 rotation/sec, 980ticks/rotation, 2000ticks/sec maximum
                    //if odom reading too large, probably wrong reading.
                    //just skip them to prevent position jump
                    //if(abs(delta_left_encoder/dt)>encoder_limit|| abs(delta_right_encoder/dt)>encoder_limit){
                    if(abs(delta_left_encoder/dt)>encoder_limit_|| abs(delta_right_encoder/dt)>encoder_limit_){
                        ROS_WARN("(MOTOR::Odom) Detect odom jump (%d,%d), re-initial motor...",
                        abs(delta_left_encoder/dt),abs(delta_right_encoder/dt));
                        initSerial();
                        leftEncoder = 0;
                        rightEncoder = 0;
                        delta_left_encoder = 0;
                        delta_right_encoder = 0;
                    }

                    //ROS_INFO("(MOTOR::Odom) right:%d, left:%d",delta_right_encoder,delta_left_encoder);
                    last_left_encoder_ = leftEncoder;
                    last_right_encoder_ = rightEncoder;

                    // distance travelled by each wheel
                    double left_dist = (delta_left_encoder/ticks_per_rot_)*2.0*wheel_radius_*PI_, 
                            right_dist = (delta_right_encoder/ticks_per_rot_)*2.0*wheel_radius_*PI_;
                    // velocity of each wheel
                    double left_vel = left_dist/dt, right_vel = right_dist/dt;
                    //compute odometry in a typical way given the velocities of the robot
                    double vel = (right_vel+left_vel)/2.0, vx = vel*cos(odom_th_), vy = vel*sin(odom_th_), vth = (right_vel-left_vel)/wheel_sep_;
                    double delta_x = vx*dt, delta_y = vy*dt, delta_th = vth*dt;
                   
                    if(reset_odom_){
                        reset_odom_ = false; 
                        odom_x_ = 0.0;
                        odom_y_ = 0.0;
                        odom_th_ = 0.0;
                        vel = 0.0;
                        vth = 0.0;
                    }
                    else{
                        odom_x_ += delta_x;
                        odom_y_ += delta_y;
                        odom_th_ += delta_th;  
                    }
                    //ROS_INFO("Linear vel: %f, Angular vel: %f", vel, vth);
                    //ROS_INFO("(MOTOR::Odom) %.5f,%.5f, %.5f",delta_x,delta_y,delta_th);
                     /*TEST REAL VELOCITY: angular speed varying
                    geometry_msgs::Twist real_cmd;
                    real_cmd.linear.x = vel;
                    real_cmd.angular.z = vth;
                    real_vel_pub_.publish(real_cmd);
                    */
                    //since all odometry is 6DOF we'll need a quaternion created from yaw
                    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th_);

                    //try robot_pose_ekf with gyro sensor
                    //first, we'll publish the transform over tf
                    geometry_msgs::TransformStamped odom_trans;
                    odom_trans.header.stamp = current_time;
                    odom_trans.header.frame_id = "odom";
                    odom_trans.child_frame_id = "base_link";
                    odom_trans.transform.translation.x = odom_x_;
                    odom_trans.transform.translation.y = odom_y_;
                    odom_trans.transform.translation.z = 0.0;
                    odom_trans.transform.rotation = odom_quat;
                    //send the transform
                    odom_broadcaster_.sendTransform(odom_trans);
                    
                    //next, we'll publish the odometry message over ROS
                    nav_msgs::Odometry odom;
                    odom.header.stamp = ros::Time::now();
                    odom.header.frame_id = "odom";
                    odom.child_frame_id = "base_link";
                    //set the position
                    odom.pose.pose.position.x = odom_x_;
                    odom.pose.pose.position.y = odom_y_;
                    odom.pose.pose.position.z = 0.0;
                    odom.pose.pose.orientation = odom_quat;
                    //set the velocity
                    odom.twist.twist.linear.x = vel;
                    odom.twist.twist.linear.y = 0.0;
                    odom.twist.twist.angular.z = vth;
                    //set covariance
                    for (int i=0;i<6;i++){
                        odom.pose.covariance[i*7] = 0.01;
                        odom.twist.covariance[i*7] = 0.01;
                    }
                    //publish the message
                    odom_pub_.publish(odom);

                    last_time = current_time;
                }
                else
                    ROS_WARN("(MOTOR::getEncoders) Got the wrong number of encoders data : %lu", encoders.size());

                r.sleep();
            }
        } 


        void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
            spdMutex_.lock();
            rec_leftSpeed_ = speed->directionL.compare("F") == 0 ? 128 + speed->velocityL : 128 - speed->velocityL;  
            rec_rightSpeed_ = speed->directionR.compare("F") == 0 ? 128 + speed->velocityR : 128 - speed->velocityR;
            spdMutex_.unlock();

            writeAndRead(std::vector<uint8_t>({0x00, 0x31, rec_leftSpeed_, 0x00, 0x32, rec_rightSpeed_})); 
        }

        bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
            reset_odom_ = true;
            return true;
        }

        /// Set the encoders to 0
        bool resetEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
            reset_encoder_ = true;
            return true;
        }

        bool motorReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
            return true;
        }


    private:
        /// Write and read informations on the serial port
        std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead = 0){
            std::vector<uint8_t> buff;
            /// Lock the mutex so no one can write at the same time
            serialMutex_.lock();
            if(serialConnection.isOpen()){
                try{
                    /// Send bytes to the MD49
                    size_t bytes_wrote = serialConnection.write(toWrite);
                    /// Read any byte that we are expecting
                    if(bytesToRead > 0)
                        serialConnection.read(buff, bytesToRead);

                    //serialConnection.flush();
                } catch (std::exception& e) {
                    ROS_ERROR("(MOTOR::exception) : %s", e.what());
                }
            } 
            else {
                ROS_WARN("(MOTOR::writeAndRead) The serial connection is not opened, something is wrong");
            }

            /// Unlock the mutex
            serialMutex_.unlock();

            return buff;
        }


        std::string motor_device_;
        bool test_encoders_, reset_odom_, reset_encoder_;
        int encoder_limit_, odom_rate_;
        double odom_x_, odom_y_, odom_th_, wheel_sep_, wheel_radius_ , ticks_per_rot_;
        uint8_t leftSpeed_, rightSpeed_, rec_leftSpeed_, rec_rightSpeed_;
        int32_t last_left_encoder_, last_right_encoder_;
        std::mutex serialMutex_, spdMutex_;
        serial::Serial serialConnection;

        tf::TransformBroadcaster odom_broadcaster_;
        ros::Time current_time, last_time;
        ros::ServiceServer motorReadySrv, resetOdomSrv, resetEncodersSrv;
        ros::Publisher odom_pub_, odom_test_pub_, encoder_pub_, real_vel_pub_;
        ros::Subscriber motorSpd_sub_;

        boost::thread *odom_thread_;
};