#include <gobot_control/gobot_diff_drive_plugin.hpp>

namespace gazebo {
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GobotDiffDrivePlugin);
    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GobotDiffDrivePlugin::GobotDiffDrivePlugin() : GazeboRosDiffDrive(){
        std::cout << "GobotDiffDrivePlugin initialised." << std::endl;
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    GobotDiffDrivePlugin::~GobotDiffDrivePlugin() {
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Load the controller
    void GobotDiffDrivePlugin::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf ) {
        std::cout << "GobotDiffDrivePlugin::Load started." << std::endl;
        // Make sure the ROS node for Gazebo has already been initalized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        GazeboRosDiffDrive::Load(_model, _sdf);

        ros::NodeHandle nh;
        this->speedService = nh.advertiseService("setSpeeds", &GobotDiffDrivePlugin::setSpeeds, this);
        this->speedPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        wheel_separation_ = std::stod(_sdf->GetElement("wheelSeparation")->GetValue()->GetAsString());

        std::cout << "GobotDiffDrivePlugin::Load launched." << std::endl;
    }

    
    bool GobotDiffDrivePlugin::setSpeeds(gobot_base::SetSpeeds::Request &req, gobot_base::SetSpeeds::Response &res){
        int reduction_coeff = 60;
        /// Wheels on the real robot are montedbackward
        double vel_r = (double) (req.directionL.compare("B") == 0 ? -1 : 1) * req.velocityL / reduction_coeff;
        double vel_l = (double) (req.directionR.compare("B") == 0 ? -1 : 1) * req.velocityR / reduction_coeff;

        double linear = (double) (vel_l + vel_r) / 2.0;
        double angular = (double) (vel_l - vel_r) / this->wheel_separation_;


        std::cout << "setSpeeds new velocity " << vel_l << " " << vel_r << " " << this->wheel_separation_ << " " << linear << " " << angular << std::endl;

        geometry_msgs::Twist twist;
        twist.linear.x = linear;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = angular;


        this->speedPublisher.publish(twist);

        return true;
    }
}