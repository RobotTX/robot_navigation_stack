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
            initialized_ = true;
        }
        else{
            ROS_ERROR("(ALLOW_BACKWARD) You should not call initialize twice on this object, doing nothing");
        }
    }

    void InitBackRecovery::runBehavior()
    {
        if(!initialized_){
            ROS_ERROR("(ALLOW_BACKWARD) This object must be initialized before runBehavior is called");
            return;
        }

        ROS_WARN("(ALLOW_BACKWARD) Init back recovery behavior started.");

        std_srvs::Empty arg;
        if(ros::service::call("/gobot_recovery/allow_teb_initBack",arg))
            ROS_WARN("(ALLOW_BACKWARD) Allow teb init with backwards motion");
        else
            ROS_ERROR("(ALLOW_BACKWARD) Failed to allow teb init with backwards motion");
    }
};