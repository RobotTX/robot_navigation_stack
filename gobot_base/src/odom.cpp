#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gobot_base/GetEncoders.h>
#include <std_srvs/Empty.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;

    if(ros::service::waitForService("resetEncoders", ros::Duration(5))){

        std_srvs::Empty arg;
        ros::service::call("resetEncoders", arg);

        ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
        tf::TransformBroadcaster odom_broadcaster;

        double x = 0.0;
        double y = 0.0;
        double th = 0.0;

        ros::Time current_time, last_time;
        current_time = ros::Time::now();
        last_time = ros::Time::now();

        int skipped = 0;

        /// Params robot dependents
        double wheel_separation;
        n.getParam("wheel_separation", wheel_separation);
        double wheel_radius;
        n.getParam("wheel_radius", wheel_radius);
        int ticks_per_rotation;
        n.getParam("ticks_per_rotation", ticks_per_rotation);

        /// The encoders should be reinitialized to 0 every time we launch odom
        int64_t last_left_encoder = 0;
        int64_t last_right_encoder = 0;

        double pi = 3.14159;

        ros::Rate r(20.0);
        while(n.ok()){

            ros::spinOnce();               // check for incoming messages

            gobot_base::GetEncoders encoders;
            if(ros::service::call("getEncoders", encoders)){
                current_time = ros::Time::now();

                double dt = (current_time - last_time).toSec();

                // difference of ticks compared to last time
                int64_t delta_left_encoder = encoders.response.leftEncoder - last_left_encoder;
                int64_t delta_right_encoder = encoders.response.rightEncoder - last_right_encoder;

                last_left_encoder = encoders.response.leftEncoder;
                last_right_encoder = encoders.response.rightEncoder;

                // distance travelled by each wheel
                double left_dist = (delta_left_encoder / (double) ticks_per_rotation) * 2 * wheel_radius * pi;
                double right_dist = (delta_right_encoder / (double) ticks_per_rotation) * 2 * wheel_radius * pi;

                // velocity of each wheel
                double left_vel = left_dist / dt;
                double right_vel = right_dist / dt;

                //compute odometry in a typical way given the velocities of the robot
                double vx = (right_vel + left_vel) / (double) 2;
                double vy = 0;
                double vth = (right_vel - left_vel) / wheel_separation;

                double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
                double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
                double delta_th = vth * dt;

/*
                std::cout << "odom : " << delta_left_encoder << " " << delta_right_encoder << " " << left_dist << " " << right_dist 
                << " " << left_vel << " " << right_vel << " " << vx << " " << vth << " " << delta_x << " " << delta_y << " " << delta_th << " " << std::endl;
*/

                x += delta_x;
                y += delta_y;
                th += delta_th;

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
                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vth;

                //publish the message
                odom_pub.publish(odom);

                last_time = current_time;
            } else {
                skipped++;
                std::cout << "(odom) couldn't call service getEncoders, skipping this odom pub, total skipped : " << skipped << std::endl;
            }

            r.sleep();
        }
    } else {
        std::cout << "(odom) waited 5 seconds for service resetEncoders" << std::endl;
    }
}