#include <gobot_recovery/allow_teb_back.h>

ros::Subscriber goalResult;

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    goalResult.shutdown();
    dynamic_reconfigure::Reconfigure config;
    dynamic_reconfigure::BoolParameter param;
    param.name = "allow_init_with_backwards_motion";
    param.value = false; 
    //Don't allow teb_local to backwards initialize
    config.request.config.bools.push_back(param);
    ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config);
    switch(msg->status.status){
        case 2:
            ROS_INFO("(ALLOW_BACKWARD) Goal PREEMPTED and disable teb_local_planner allow_init_with_backwards_motion.");
            break;
        case 3:
            ROS_INFO("(ALLOW_BACKWARD) Goal PREEMPTED and disable teb_local_planner allow_init_with_backwards_motion.");
            break;
        case 4:
            ROS_INFO("(ALLOW_BACKWARD) Goal ABORTED and disable teb_local_planner allow_init_with_backwards_motion.");
            break;
        default:
            ROS_ERROR("(ALLOW_BACKWARD) Unknown goal status %d and disable teb_local_planner allow_init_with_backwards_motion",msg->status.status);
            break;
    }
}

bool allowTebSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ros::NodeHandle nh;
    dynamic_reconfigure::Reconfigure config;
    dynamic_reconfigure::BoolParameter param;
    param.name = "allow_init_with_backwards_motion";
    param.value = true;    
    //allow teb_local to backwards initialize
    config.request.config.bools.push_back(param);
    if(ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config)){
        ROS_INFO("(ALLOW_BACKWARD) Enable teb_local_planner allow_init_with_backwards_motion.");
        goalResult = nh.subscribe("/move_base/result",1,goalResultCallback);
        return true;
    }
    else{
        ROS_ERROR("(ALLOW_BACKWARD) Unable to set teb_local_planner allow_init_with_backwards_motion");
        return false;
    }
}

void mySigintHandler(int sig){  
    goalResult.shutdown();
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wheels");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);
    
    ros::ServiceServer allowTebSrv = nh.advertiseService("/gobot_recovery/allow_teb_initBack",allowTebSrvCallback);

    ros::spin();
    return 0;
}
