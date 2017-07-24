#ifndef GOBOT_DIFF_DRIVE_PLUGIN_HPP
#define GOBOT_DIFF_DRIVE_PLUGIN_HPP

#include <ros/ros.h>
#include <gazebo_plugins/gazebo_ros_diff_drive.h>
#include <gobot_base/SetSpeeds.h>

namespace gazebo {
    class GobotDiffDrivePlugin : public GazeboRosDiffDrive {
        public:
            GobotDiffDrivePlugin();
            virtual ~GobotDiffDrivePlugin();
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            bool setSpeeds(gobot_base::SetSpeeds::Request &req, gobot_base::SetSpeeds::Response &res);

        private:
            ros::ServiceServer speedService;
            ros::Publisher speedPublisher;
            double wheel_separation_;
    };
}
#endif
