#include <gobot_recovery/init_back_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(init_back_recovery, InitBackRecovery, init_back_recovery::InitBackRecovery, nav_core::RecoveryBehavior)

namespace init_back_recovery {

InitBackRecovery::InitBackRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false) {} 

    void InitBackRecovery::initialize(std::string name, tf::TransformListener* tf, 
            costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
    {

        if(!initialized_)
        {
            name_=name;
            ros::NodeHandle private_nh("~/" + name_);

            private_nh.param("allow_time", allow_time_, 0);
            initialized_ = true;
        }
        else{
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    void InitBackRecovery::runBehavior()
    {
        if(!initialized_){
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        ROS_WARN("Init back recovery behavior started.");

        std_srvs::Empty arg;
        if(ros::service::call("/gobot_recovery/Allow_Teb_InitBack",arg))
            ROS_WARN("Allow teb init with backwards motion.");
        else
            ROS_ERROR("Failed to allow teb init with backwards motion.");

        ros::Duration(0.1).sleep();
    }
};