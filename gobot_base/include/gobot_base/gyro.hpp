//ros headers
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
//c++ headers
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <numeric> 
#include <serial/serial.h>
#include <thread>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <gobot_msg_srv/robot_msgs.h>


#define PI_ 3.1415926

class GyroClass {
    public:
        GyroClass(){ 
            ros::NodeHandle nh;
            //load parameters
            nh.getParam("GYRO_PORT", gyro_device_);
            
            if(!initSerial()){
                exit(1);
            }

            gyro_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_data", 1);
            height_pub_ = nh.advertise<std_msgs::Float32>("/height_data", 1);

            //set up the planner's thread
            gyro_thread_ = new boost::thread(&GyroClass::gyroThread, this);

        }

        ~GyroClass(){
            gyro_thread_->interrupt();
            gyro_thread_->join();
            delete gyro_thread_;

            try{
                serialConnection.write(std::vector<uint8_t>({0xFF, 0xAA, 0x02, 0x00, 0x00}));
                serialConnection.flush();
                serialConnection.close();
            } catch (std::exception& e) {
                std::cout<<"(GYRO::Shutdown) exception : "<<e.what()<<std::endl;
            }
        }

        /// Initialize the serial connection
        bool initSerial(){
            /// Get the port in which our device is connected
            ROS_INFO("(GYRO::initSerial) GYRO port : %s", gyro_device_.c_str());

            serialConnection.setPort(gyro_device_);
            serialConnection.setBaudrate(9600);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            serialConnection.setTimeout(timeout);
            serialConnection.close();
            serialConnection.open();
            serialConnection.write(std::vector<uint8_t>({0xFF, 0xAA, 0x02, 0x46, 0x02}));
            serialConnection.flush();
            ros::Duration(0.1).sleep();
            return true;
        }


        void gyroThread(){
            findStartByte();

            ros::Rate loop(10);
            //checking procedure
            while(ros::ok()){
                std::vector<uint8_t> buff;
                serialConnection.read(buff, 44);
                if(buff.size()==44){
                    int start_address = buff.at(1), end_address = buff.at(34);
                    if(start_address==81 && end_address==89){
                        sensor_msgs::Imu imu_data;
                        imu_data.header.stamp = ros::Time::now();
                        imu_data.header.frame_id = "base_link";

                        int16_t temp;

                        temp = (buff.at(3)<<8)|buff.at(2);
                        imu_data.linear_acceleration.x=16*9.8*temp/32768.0;
                        temp = (buff.at(5)<<8)|buff.at(4);
                        imu_data.linear_acceleration.y=16*9.8*temp/32768.0;
                        temp = (buff.at(7)<<8)|buff.at(6);
                        imu_data.linear_acceleration.z=16*9.8*temp/32768.0;
                        imu_data.linear_acceleration_covariance = {0.1, 0.0, 0.0,
                                                                0.0, 0.1, 0.0,
                                                                0.0, 0.0, 0.1};
                        temp = (buff.at(9)<<8)|buff.at(8);
                        double temperature = temp/100.0;

                        temp = (buff.at(14)<<8)|buff.at(13);
                        imu_data.angular_velocity.x = (PI_*2000*temp/32768.0)/180.0;
                        temp = (buff.at(16)<<8)|buff.at(15);
                        imu_data.angular_velocity.y = (PI_*2000*temp/32768.0)/180.0;
                        temp = (buff.at(18)<<8)|buff.at(17);
                        imu_data.angular_velocity.z = (PI_*2000*temp/32768.0)/180.0;
                        imu_data.angular_velocity_covariance = {0.05, 0, 0,
                                                                0, 0.05, 0,
                                                                0, 0, 0.05};
                        int32_t height = (buff.at(31)<<24)|(buff.at(30)<<16)|(buff.at(29)<<8)|buff.at(28);
                        std_msgs::Float32 height_data;
                        height_data.data = height/100.0;

                        temp = (buff.at(36)<<8)|buff.at(35);
                        imu_data.orientation.w = temp/32768.0;
                        temp = (buff.at(38)<<8)|buff.at(37);
                        imu_data.orientation.x = temp/32768.0;
                        temp = (buff.at(40)<<8)|buff.at(39);
                        imu_data.orientation.y = temp/32768.0;
                        temp = (buff.at(42)<<8)|buff.at(41);
                        imu_data.orientation.z = temp/32768.0;
                        imu_data.orientation_covariance = {0.01, 0, 0,
                                                           0, 0.01, 0,
                                                           0, 0, 0.01};
                        double yaw = tf::getYaw(tf::Quaternion(imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w));
                        gyro_pub_.publish(imu_data);
                        height_pub_.publish(height_data);

                        /*
                        std::cout<<"Data length: "<<buff.size()<<". Start address: " <<start_address<<". End address: " <<end_address<<std::endl;
                        std::cout<<"Temperature: "<<temperature<<" *C"<<std::endl;
                        std::cout<<"Height: "<<height_data.data<<" m"<<std::endl;
                        std::cout << "Yaw: "<<yaw*180/PI_<<" degrees"<<std::endl;
                        std::cout<<std::endl;
                        */
                    }
                    else{
                        findStartByte();
                    }
                }
                else{
                    std::cout<<"Wrong data size received!"<<std::endl;
                }
                loop.sleep();
                serialConnection.flushInput();
            }
        } 

    private:
        void findStartByte(){
            std::cout<<"Start to find beginning byte..."<<std::endl;
            int pre_data = 0, count=0;
            while(ros::ok()){
                std::vector<uint8_t> buff;
                serialConnection.read(buff, 1);
                int data = buff.at(0);
                if(pre_data==85 && data==89){
                    count++;
                    if(count == 3){
                        serialConnection.read(buff, 9);
                        break;
                    }
                }
                pre_data = data;
            }
        }

        std::string gyro_device_;
        serial::Serial serialConnection;

        ros::Time output_time_;
        ros::Publisher gyro_pub_, height_pub_;

        boost::thread *gyro_thread_;
};