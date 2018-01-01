#ifndef INIT_BACK_RECOVERY_H_
#define INIT_BACK_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>


namespace init_back_recovery{
  class InitBackRecovery : public nav_core::RecoveryBehavior {
    public:

      InitBackRecovery();

      void initialize(std::string name, tf::TransformListener* tf, 
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      void runBehavior();


    protected:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      int allow_time_;
  };
};
#endif 