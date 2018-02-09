#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gobot_msg_srv/GetEncoders.h>
#include <gobot_msg_srv/OdomTestMsg.h>
#include <gobot_msg_srv/GetGobotStatus.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

std_srvs::Empty arg;

void mySigintHandler(int sig)
{   
    ros::shutdown();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);

    //Startup begin
    ROS_INFO("(Odom) Waiting for MD49 to be ready...");
    ros::service::waitForService("/gobot_startup/motor_ready", ros::Duration(60.0));
    ROS_INFO("(Odom) MD49 is ready.");
    //Startup end

    //ros::ServiceServer getSpeedsSrv = n.advertiseService("/gobot_motor/getRealSpeeds", getRealSpeeds);
    /// The encoders should be reinitialized to 0 every time we launch odom
    int32_t last_left_encoder = 0;
    int32_t last_right_encoder = 0;
    
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher odom_test_pub = n.advertise<gobot_msg_srv::OdomTestMsg>("odom_test", 50);
    ros::Publisher real_vel_pub = n.advertise<geometry_msgs::Twist>("real_vel", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    last_time = ros::Time::now();

    int skipped = 0;

    /// Params robot dependents
    double wheel_separation;
    n.getParam("wheel_separation", wheel_separation);
    double wheel_radius;
    n.getParam("wheel_radius", wheel_radius);
    double ticks_per_rotation;
    n.getParam("ticks_per_rotation", ticks_per_rotation);
    int ODOM_RATE=10;
    n.getParam("ODOM_RATE", ODOM_RATE);

    double pi = 3.1415926;

    ros::Rate r(ODOM_RATE); //20
    ros::service::call("/gobot_motor/resetEncoders", arg);
    while(ros::ok()){

        // check for incoming messages
        ros::spinOnce();

        gobot_msg_srv::GetEncoders encoders;
        if(ros::service::call("/gobot_motor/getEncoders", encoders)){
            current_time = ros::Time::now();

            double dt = (current_time - last_time).toSec();

            // difference of ticks compared to last time
            //122rpm, 2 rotation/sec, 980ticks/rotation, 2000ticks/sec maximum
            //if odom reading too large, probably wrong reading.
            //just skip them to prevent position jump
            if(abs(encoders.response.leftEncoder - last_left_encoder)>500|| abs(encoders.response.rightEncoder - last_right_encoder)>500){
                ROS_WARN("(Odom) Detect odom jump (%d,%d), re-initial motor...",
                abs(encoders.response.leftEncoder - last_left_encoder),abs(encoders.response.rightEncoder - last_right_encoder));
                ros::service::call("/gobot_motor/initialMotor", arg);
                if(ros::service::call("/gobot_motor/getEncoders", encoders))
                last_left_encoder = 0;
                last_right_encoder = 0;
                last_time = current_time;
                continue;
            }

            int32_t delta_left_encoder = encoders.response.leftEncoder - last_left_encoder;
            int32_t delta_right_encoder = encoders.response.rightEncoder - last_right_encoder;

            //ROS_INFO("right:%zu, left:%zu",delta_right_encoder,delta_left_encoder);
            last_left_encoder = encoders.response.leftEncoder;
            last_right_encoder = encoders.response.rightEncoder;

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
            odom.header.stamp = current_time;
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

            /*
            /// some test
            gobot_msg_srv::OdomTestMsg odomTest;
            odomTest.x = x;
            odomTest.y = y;
            odomTest.yaw = th;
            odom_test_pub.publish(odomTest);
            */


            last_time = current_time;
        } 
        else {
            skipped++;
            ROS_WARN("(odom) couldn't call service /gobot_motor/getEncoders, skipping this odom pub, total skipped : %d", skipped);
        }

        r.sleep();
    }

    return 0;
}