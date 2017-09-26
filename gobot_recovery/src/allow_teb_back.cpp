#include <gobot_recovery/allow_teb_back.hpp>

bool allow_teb = false;

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if(allow_teb){
        allow_teb=false;
        dynamic_reconfigure::Reconfigure config;
        dynamic_reconfigure::BoolParameter param;
        param.name = "allow_init_with_backwards_motion";
        param.value = false; 
        //Don't allow teb_local to backwards initialize
        config.request.config.bools.push_back(param);
        ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config);
        switch(msg->status.status){
            case 2:
                ROS_INFO("Goal PREEMPTED and disable teb_local_planner allow_init_with_backwards_motion.");
                break;
            case 3:
                ROS_INFO("Goal PREEMPTED and disable teb_local_planner allow_init_with_backwards_motion.");
                break;
            case 4:
                ROS_INFO("Goal ABORTED and disable teb_local_planner allow_init_with_backwards_motion.");
                break;
            default:
                ROS_ERROR("Unknown goal status %d and disable teb_local_planner allow_init_with_backwards_motion.",msg->status.status);
                break;
        }
    }
}

bool allowTebCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    dynamic_reconfigure::Reconfigure config;
    dynamic_reconfigure::BoolParameter param;
    param.name = "allow_init_with_backwards_motion";
    param.value = true;    
    //allow teb_local to backwards initialize
    config.request.config.bools.push_back(param);
    if(ros::service::call("/move_base/TebLocalPlannerROS/set_parameters",config)){
        ROS_INFO("Enable teb_local_planner allow_init_with_backwards_motion.");
        allow_teb = true;
        return true;
    }
    else
        ROS_ERROR("Unable to set teb_local_planner allow_init_with_backwards_motion.");
        return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wheels");
    ros::NodeHandle nh;

    ros::ServiceServer allowTebSrv = nh.advertiseService("/gobot_recovery/Allow_Teb_InitBack",allowTebCallback);
    ros::Subscriber goalResult = nh.subscribe("/move_base/result",1,goalResultCallback);

    ros::spin();
    return 0;
}
