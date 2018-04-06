#include <gobot_base/wheels.hpp>

bool testEncoder = false;
uint8_t leftSpeed_=128, rightSpeed_=128, rec_leftSpeed_=128,rec_rightSpeed_=128;

std::mutex serialMutex, dataMutex;
std::string MD49device;
serial::Serial serialConnection;
int ACC_RATE = 0,ROLLING_WINDOW_SIZE = 5;

//odom related data
int32_t last_left_encoder = 0, last_right_encoder = 0;
double x = 0.0, y = 0.0, th = 0.0, pi = 3.1415926;
int encoder_limit = 3000;
ros::Time current_time, last_time;

 //odom info
int ODOM_RATE=10;
double wheel_separation,wheel_radius,ticks_per_rotation;

/// Write and read informations on the serial port
std::vector<uint8_t> writeAndRead(std::vector<uint8_t> toWrite, int bytesToRead){
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
		    ROS_ERROR("(Wheels) exception : %s", e.what());
	    }
    } 
    else {
        ROS_WARN("(wheels::writeAndRead) The serial connection is not opened, something is wrong");
    }

    /// Unlock the mutex
    serialMutex.unlock();

    return buff;
}

/// get the output of the given system command
std::string getStdoutFromCommand(std::string cmd) {
    std::string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}

/// Just to test, we get the encoders and print them
bool testEncoders(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    testEncoder = true;
    return true;
}

bool stopTests(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    testEncoder = false;
    return true;
}

bool resetOdom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    dataMutex.lock();
    writeAndRead(std::vector<uint8_t>({0x00, 0x35}));
    last_left_encoder = 0;
    last_right_encoder = 0;
    x=0;
    y=0;
    th=0;
    last_time = current_time;
    dataMutex.unlock();
    return true;
}

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
    if(speed->velocityL <= 127)
        rec_leftSpeed_ = speed->directionL.compare("F") == 0 ? 128 + speed->velocityL : 128 - speed->velocityL;  
    else
        rec_leftSpeed_ = speed->directionL.compare("F") == 0 ? 255 : 0;

    if(speed->velocityR <= 127)
        rec_rightSpeed_ = speed->directionR.compare("F") == 0 ? 128 + speed->velocityR : 128 - speed->velocityR;
    else
        rec_rightSpeed_ = speed->directionL.compare("F") == 0 ? 255 : 0;
    
    //leftSpeed_ = rec_leftSpeed_;
    //rightSpeed_ = rec_rightSpeed_;
    //ROS_INFO("(wheels::setSpeeds) Data : %s %d %s %d", req.directionL.c_str(), (int) req.velocityL, req.directionR.c_str(), (int) req.velocityR);
    //ROS_INFO("(wheels::setSpeeds) Left:%d    Right:%d", leftSpeed_,rightSpeed_);
    //writeAndRead(std::vector<uint8_t>({0x00, 0x31, leftSpeed_, 0x00, 0x32, rightSpeed_})); 
}

bool motorReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return true;
}


bool initSerial(){
    /// Get the port in which our device is connected
    std::string port = MD49device;
    ROS_INFO("(wheels::initSerial) MD49 port : %s", port.c_str());

    serialMutex.lock();
    if(serialConnection.isOpen())
        serialConnection.close();

    // Set the serial port, baudrate and timeout in milliseconds
    serialConnection.setPort(port);
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
        ROS_INFO("(wheels::initSerial) Established connection to MD49.");
        serialMutex.unlock();
        return true;
    } 
    else{
        serialMutex.unlock();
        return false;
    }
}

void initParams(ros::NodeHandle &nh){
    nh.getParam("MD49device", MD49device);
    nh.getParam("wheel_separation", wheel_separation);
    nh.getParam("wheel_radius", wheel_radius);
    nh.getParam("ticks_per_rotation", ticks_per_rotation);
    nh.getParam("ODOM_RATE", ODOM_RATE);
    nh.getParam("ACC_RATE", ACC_RATE);
    nh.getParam("ROLLING_WINDOW_SIZE", ROLLING_WINDOW_SIZE);
}

void mySigintHandler(int sig){   
    serialMutex.lock();
    try{
        //set speed to 0 when shutdown
        serialConnection.write(std::vector<uint8_t>({0x00, 0x31, 0x80, 0x00, 0x32, 0x80}));
        serialConnection.close();
	} catch (std::exception& e) {
		ROS_ERROR("(Wheels) Shutdown exception : %s", e.what());
	}
    serialMutex.unlock();
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wheels");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    initParams(nh);

    if(initSerial()){
        //Startup begin
        ros::service::waitForService("/gobot_status/set_gobot_status", ros::Duration(60.0));
        ros::ServiceServer motorReadySrv = nh.advertiseService("/gobot_startup/motor_ready", motorReadySrvCallback);
        //Startup end

        ros::ServiceServer resetOdomSrv = nh.advertiseService("/gobot_motor/resetOdom", resetOdom);
        ros::ServiceServer testEncodersSrv = nh.advertiseService("/gobot_test/testEncoders", testEncoders);
        ros::ServiceServer stopTestsSrv = nh.advertiseService("/gobot_test/stopTestEncoder", stopTests);

        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
        ros::Publisher odom_test_pub = nh.advertise<gobot_msg_srv::OdomTestMsg>("/odom_test", 50);
        ros::Publisher encoder_pub = nh.advertise<gobot_msg_srv::EncodersMsg>("/encoders", 50);
        ros::Publisher real_vel_pub = nh.advertise<geometry_msgs::Twist>("/real_vel", 50);
        
        ros::Subscriber motorSpd_sub = nh.subscribe("/gobot_motor/motor_speed", 1, motorSpdCallback);
        
        tf::TransformBroadcaster odom_broadcaster;
        last_time = ros::Time::now();

        //rolling window mean
        std::vector<uint8_t> left_vel_mean_(ROLLING_WINDOW_SIZE,leftSpeed_), right_vel_mean_(ROLLING_WINDOW_SIZE,rightSpeed_);

        ros::AsyncSpinner spinner(3); // Use 3 threads
        spinner.start();

        ros::Rate r(ODOM_RATE);
        while(ros::ok()){
            dataMutex.lock();
            std::vector<uint8_t> encoders = writeAndRead(std::vector<uint8_t>({0x00, 0x25}), 8);

            if(encoders.size() == 8){
                int32_t leftEncoder = (encoders.at(0) << 24) + (encoders.at(1) << 16) + (encoders.at(2) << 8) + encoders.at(3);
                int32_t rightEncoder = (encoders.at(4) << 24) + (encoders.at(5) << 16) + (encoders.at(6) << 8) + encoders.at(7);
                current_time = ros::Time::now();
                double dt = (current_time - last_time).toSec();

                int32_t delta_left_encoder = leftEncoder - last_left_encoder;
                int32_t delta_right_encoder = rightEncoder - last_right_encoder;

                // difference of ticks compared to last time
                //122rpm, 2 rotation/sec, 980ticks/rotation, 2000ticks/sec maximum
                //if odom reading too large, probably wrong reading.
                //just skip them to prevent position jump
                //if(abs(delta_left_encoder/dt)>encoder_limit|| abs(delta_right_encoder/dt)>encoder_limit){
                if(abs(delta_left_encoder/dt)>encoder_limit|| abs(delta_right_encoder/dt)>encoder_limit){
                    ROS_WARN("(wheels::Odom) Detect odom jump (%d,%d), re-initial motor...",
                    abs(delta_left_encoder/dt),abs(delta_right_encoder/dt));
                    initSerial();
                    last_left_encoder = 0;
                    last_right_encoder = 0;
                }
                else{
                    //ROS_INFO("right:%d, left:%d",delta_right_encoder,delta_left_encoder);
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
                    //ROS_INFO("Linear vel:%.3f, Angular vel:%.3f",vel,vth);

                    double delta_x = vx * dt;
                    double delta_y = vy * dt;
                    double delta_th = vth * dt;

                    /*TEST REAL VELOCITY: angular speed varying
                    geometry_msgs::Twist real_cmd;
                    real_cmd.linear.x = vel;
                    real_cmd.angular.z = vth;
                    real_vel_pub.publish(real_cmd);
                    */

                    x += delta_x;
                    y += delta_y;
                    th += delta_th;  
                    //ROS_INFO("%.5f,%.5f, %.5f",delta_x,delta_y,delta_th);
                    
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
                    /*
                    /// some test
                    gobot_msg_srv::OdomTestMsg odomTest;
                    odomTest.x = x;
                    odomTest.y = y;
                    odomTest.yaw = th;
                    odom_test_pub.publish(odomTest);
                    */
                    // check for incoming messages
                }

                last_time = current_time;
            }
            else
                ROS_WARN("(wheels::getEncoders) Got the wrong number of encoders data : %lu", encoders.size());
            
            dataMutex.unlock();

            //adopt rolling window mean to smooth motor speed
            left_vel_mean_.erase(left_vel_mean_.begin());
            left_vel_mean_.push_back(rec_leftSpeed_);
            right_vel_mean_.erase(right_vel_mean_.begin());
            right_vel_mean_.push_back(rec_rightSpeed_);

            //if command is stop motor, directly apply the value
            if(rec_leftSpeed_==128 && rec_rightSpeed_==128){
                leftSpeed_ = rec_leftSpeed_;
                rightSpeed_ = rec_rightSpeed_;
            }
            //else, apply rolling window mean
            else{
                leftSpeed_ = std::accumulate(left_vel_mean_.begin(),left_vel_mean_.end(),0)/left_vel_mean_.size();
                rightSpeed_ = std::accumulate(right_vel_mean_.begin(),right_vel_mean_.end(),0)/right_vel_mean_.size();
            }
            writeAndRead(std::vector<uint8_t>({0x00, 0x31, leftSpeed_, 0x00, 0x32, rightSpeed_})); 
            //ROS_INFO("Left mean vel:%d, Right mean vel:%d",leftSpeed_,rightSpeed_);

            r.sleep();
        }
    } 
    else
        ROS_INFO("(wheels::main) Could not open the serial communication");
    
    ros::waitForShutdown();
    return 0;
}
