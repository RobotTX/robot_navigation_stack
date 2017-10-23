#include <gobot_recovery/initialpose_estimation.h>


double cov_x=0.0,cov_y=0.0,cov_yaw=0.0,initial_cov_xy=0.02,initial_cov_yaw=0.02;
//Make robot rotate 360 degrees
double rot_vel = 0.4, rot_time = 150;
double last_pos_x=0.0,last_pos_y=0.0,last_pos_yaw=0.0,last_ang_x=0.0,last_ang_y=0.0,last_ang_z=0.0,last_ang_w=0.0;
double home_pos_x=0.0,home_pos_y=0.0,home_pos_yaw=0.0,home_ang_x=0.0,home_ang_y=0.0,home_ang_z=0.0,home_ang_w=0.0;
double rosparam_x=0.0,rosparam_y=0.0,rosparam_yaw=0.0,rosparam_cov_x=0.0,rosparam_cov_y=0.0,rosparam_cov_yaw=0.0;

bool running = false,found_pose=false;

std_srvs::Empty empty_srv;

ros::Publisher vel_pub,foundPose_pub,initial_pose_publisher,goal_pub,lost_pub;
std::string lastPoseFile;
gobot_msg_srv::GetGobotStatus get_gobot_status;
ros::ServiceClient getGobotStatusSrv;

bool evaluatePose(int type){
    //Evaluate ROS server pose 
    if(type==0){
        if(rosparam_cov_x<COV_XY_T && rosparam_cov_y<COV_XY_T && rosparam_cov_yaw<COV_YAW_T)
            return true;
        else
            return false;
    }
    //Evaluate Last pose compared to Home pose
    else if(type==1){
        if(home_pos_x!=0 || home_pos_y!=0 || home_ang_x!=0 || home_ang_y!=0 || home_ang_z!=0 || home_ang_w!=0)
            if(std::abs(home_pos_x-last_pos_x)<0.5 && std::abs(home_pos_y-last_pos_y)<0.5 && std::abs(home_pos_yaw-last_pos_yaw)<PI*11/180)
                return true;
        
        return false;
    }
}

bool rotateFindPose(double rot_v,double rot_t){
    double dt=0.0;
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z=rot_v;
    //rotate for time rot_time until find small covariance
    ros::Rate loop_rate(4);
    ros::Time last_time = ros::Time::now();
    //clear costmap before rotating
    ros::service::call("/move_base/clear_costmaps",empty_srv);
    while(running && dt<rot_t && ros::ok()){
        dt=(ros::Time::now() - last_time).toSec();
        //ROS_INFO("I am finding my pose in the map...covariance:xy=%.3f,yaw=%.3f",cov_xy,cov_yaw);
        vel_pub.publish(vel);
        if(cov_x<COV_XY_T && cov_y<COV_XY_T && cov_yaw<COV_YAW_T){
            ROS_INFO("Spent %.3f seconds to localize robot pose in the map.",dt);
            return true;
        }
        loop_rate.sleep();
    }
    vel.linear.x=0.0;
    vel.angular.z=0.0;
    vel_pub.publish(vel);

    return false;
}

void findPoseResult(int status){
    //Give stm32 some time before changing LED
    std_msgs::Int8 result;
    result.data = status;
    //Publish result
    foundPose_pub.publish(result);
}


void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w,const double cov1,const double cov2){

    if(position_x != 0 || position_y != 0 || angle_x != 0 || angle_y != 0 || angle_z != 0 || angle_w != 0){
        geometry_msgs::PoseWithCovarianceStamped initialPose;
        initialPose.header.frame_id = "map";
        initialPose.header.stamp = ros::Time::now();
        initialPose.pose.pose.position.x = position_x;
        initialPose.pose.pose.position.y = position_y;
        initialPose.pose.pose.orientation.x = angle_x;
        initialPose.pose.pose.orientation.y = angle_y;
        initialPose.pose.pose.orientation.z = angle_z;
        initialPose.pose.pose.orientation.w = angle_w;
        //x-xy-y,yaw-yaw
        initialPose.pose.covariance[0] = cov1;
        initialPose.pose.covariance[7] = cov1;
        initialPose.pose.covariance[35] = cov2;
        
        // we wait for amcl to launch
        ros::Duration(2.0).sleep();

        initial_pose_publisher.publish(initialPose);
        //ROS_INFO("(initial_pose_publisher) initialpose published.");
    } else
        ROS_ERROR("(initial_pose_publisher) got a wrong position (robot probably got no home)");
}

bool initializeHomeSrcCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    gobot_msg_srv::IsCharging arg;
    ros::service::call("/gobot_status/charging_status",arg);
    if(arg.response.isCharging){
        //get pose
        getPose();
        publishInitialpose(home_pos_x,home_pos_y,home_ang_x,home_ang_y,home_ang_z,home_ang_w,initial_cov_xy,initial_cov_yaw);
        ROS_INFO("Robot is in the charing station");
        found_pose=true;
    }
    return true;
}

bool initializePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(!running){
        std::thread([](){
            int current_stage=START_STAGE;
            gobot_msg_srv::IsCharging arg;
            running = true;
            found_pose=false;

            getGobotStatusSrv.call(get_gobot_status);
            //pause when goal active or goal reached(may wait for human action)
            if(get_gobot_status.response.status==5){
                ros::service::call("/gobot_command/pause_path",empty_srv);
                ROS_INFO("Cancelled active goal to proceed gobalo localization.");
            }
            else if(get_gobot_status.response.status==15){
                ROS_INFO("Stop robot home.");
                ros::service::call("/gobot_command/stopGoDock",empty_srv);
            }

            //get pose
            getPose();

            while(current_stage!=COMPLETE_STAGE && running && ros::ok()){
                switch(current_stage){
                    case START_STAGE:
                        ROS_INFO("Start finding initial pose...");
                        //start to localize
                        findPoseResult(START_FOUND);
                        current_stage=CHARGING_STAGE;
                        break;

                    case CHARGING_STAGE:
                        ros::service::call("/gobot_status/charging_status",arg);
                        //if robot is charging, it is in CS station 
                        if(arg.response.isCharging || evaluatePose(1)){
                            publishInitialpose(home_pos_x,home_pos_y,home_ang_x,home_ang_y,home_ang_z,home_ang_w,initial_cov_xy,initial_cov_yaw);
                            ROS_INFO("Robot is in the charing station");
                            found_pose=true;
                            current_stage=COMPLETE_STAGE;
                        }
                        else{
                            ROS_WARN("Robot is not in the charging station");
                            current_stage=ROSPARAM_POSE_STAGE;
                        }
                        break;

                    case ROSPARAM_POSE_STAGE:
                        ROS_INFO("Try ROS server pose...");
                        if(evaluatePose(0)){
                            ROS_INFO("Robot is near the rosparam server pose.");
                            found_pose=true;
                            current_stage=COMPLETE_STAGE;
                        }
                        else{
                            ROS_WARN("ROS server pose is not reliable");
                            current_stage=LAST_POSE_STAGE;
                        }
                        break;

                    case LAST_POSE_STAGE:
                        //try the pose from last stop pose
                        publishInitialpose(last_pos_x,last_pos_y,last_ang_x,last_ang_y,last_ang_z,last_ang_w,initial_cov_xy,initial_cov_yaw);
                        ROS_INFO("Robot is near the last stop pose.");
                        found_pose=true;
                        current_stage=COMPLETE_STAGE;
                        break;

                    default:
                        current_stage=COMPLETE_STAGE;
                        break;
                }
            }
            //clear costmap after finish
            ros::service::call("/move_base/clear_costmaps",empty_srv);
            if(found_pose){
                findPoseResult(FOUND);
                //Update and record last pose when found
                ros::service::call("/request_nomotion_update",empty_srv);

                 //Startup begin
                gobot_msg_srv::SetGobotStatus set_gobot_status;
                set_gobot_status.request.status = -1;
                set_gobot_status.request.text ="FOUND_POSE";
                ros::service::call("/gobot_status/set_gobot_status",set_gobot_status);
                //Startup end
            }
            running = false;
        }).detach();
        return true;
    }
    else{
        ROS_ERROR("Localization is running by other request and not complete yet. Call /gobot_recovery/stop_globalize_pose service to stop it");
        return false;
    }
}

bool globalizePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //check whether have active goal, and cancel it if have
    if(!running){
        std::thread([](){
            int current_stage=START_STAGE;
            running = true;
            found_pose=false;

            getGobotStatusSrv.call(get_gobot_status);
            if(get_gobot_status.response.status==5){
                ros::service::call("/gobot_command/pause_path",empty_srv);
                ROS_INFO("Cancelled active goal to proceed gobalo localization.");
            }
            else if(get_gobot_status.response.status==15){
                ROS_INFO("Stop robot home.");
                ros::service::call("/gobot_command/stopGoDock",empty_srv);
            }
            
            while(current_stage!=COMPLETE_STAGE && running && ros::ok()){
                switch(current_stage){
                    case START_STAGE:
                        ROS_INFO("Start globalizing pose...");
                        //start to localize
                        findPoseResult(START_FOUND);
                        current_stage=GLOBAL_POSE_STAGE;
                        break;

                    case GLOBAL_POSE_STAGE:
                        if(ros::service::call("/global_localization",empty_srv)){
                            ros::Duration(3.0).sleep();
                            found_pose=rotateFindPose(rot_vel,rot_time);
                            if (found_pose){
                                ROS_INFO("Found robot pose after global initialization.");
                            }
                            else if(running){
                                ROS_WARN("Unable to find robot pose in the map after global initialization");
                                ROS_WARN("Suggest to move robot to charging station and restart");
                                //failed to localize
                                findPoseResult(NOT_FOUND);
                            }
                        }
                        else{
                            ROS_ERROR("Failed to reset particles");
                        }
                        current_stage=COMPLETE_STAGE;
                        break;

                    default:
                        current_stage=COMPLETE_STAGE;
                        break;
                }
            }

            //clear costmap after finish
            ros::service::call("/move_base/clear_costmaps",empty_srv);
            if(found_pose){
                findPoseResult(FOUND);
                //Update and record last pose when found
                ros::service::call("/request_nomotion_update",empty_srv);
            }
            running = false;
        }).detach();
        return true;
    }
    else{
        ROS_ERROR("Localization is running by other request and not complete yet. Call /gobot_recovery/stop_globalize_pose service to stop it");
        return false;
    }
}

bool stopGlobalizePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(running){
        running = false;
        ROS_INFO("Stop finding initial pose.");
        //cancel localization
        if(!found_pose)
            findPoseResult(CANCEL_FOUND);
    }
    else{
        ROS_WARN("Robot is not finding initial pose");
    }
    return true;
}

void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    cov_x=msg->pose.covariance[0];
    cov_y=msg->pose.covariance[7];
    cov_yaw=msg->pose.covariance[35];

    //ROS_INFO("cov_x:%.4f,cov_y:%.4f,cov_yaw:%.4f",cov_x,cov_y,cov_yaw);
    //Write lastest amcl_pose to file
    if(found_pose){
        std_msgs::Int8 lost;
        if((cov_x > 10*initial_cov_xy && cov_y > 10*initial_cov_xy) || cov_yaw > 10*initial_cov_yaw){
            gobot_msg_srv::IsCharging arg;
            ROS_ERROR("Big covariance in the amcl pose");
            if(ros::service::call("/gobot_status/charging_status", arg) && arg.response.isCharging){
                ROS_ERROR("Found gobot is charging. Set its pose to home");
                publishInitialpose(home_pos_x,home_pos_y,home_ang_x,home_ang_y,home_ang_z,home_ang_w,initial_cov_xy,initial_cov_yaw);
                lost.data=0;
            }
            else{
                lost.data = 1;         
            }
        }
        else{
            lost.data=0;
            std::ofstream ofs(lastPoseFile, std::ofstream::out | std::ofstream::trunc);
            if(ofs.is_open()){
                ofs << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.orientation.x<<" "<< msg->pose.pose.orientation.y<<" "<< msg->pose.pose.orientation.z<<" "<< msg->pose.pose.orientation.w;
                ofs.close();
            } 
            else
                ROS_ERROR("Could not open the file %s", lastPoseFile.c_str());
        }
        lost_pub.publish(lost);
    }
}

bool goHomeSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //get pose
    getPose();

    getGobotStatusSrv.call(get_gobot_status);
    //pause when goal active or goal reached(may wait for human action)
    if(get_gobot_status.response.status==5){
        ros::service::call("/gobot_command/pause_path",empty_srv);
        ROS_INFO("Cancelled active goal to go home.");
    }
    else if(get_gobot_status.response.status==15){
        ROS_INFO("Stop robot home.");
        ros::service::call("/gobot_command/stopGoDock",empty_srv);
    }

    ROS_INFO("Go Home, sweet home.");
    geometry_msgs::PoseStamped home;
    home.header.frame_id = "map";
    home.header.stamp = ros::Time::now();
    home.pose.position.x=home_pos_x+0.5;
    home.pose.position.y=home_pos_y+0.5;
    home.pose.position.z=0.0;
    home.pose.orientation.x=home_ang_x;
    home.pose.orientation.y=home_ang_y;
    home.pose.orientation.z=home_ang_z;
    home.pose.orientation.w=home_ang_w;
    goal_pub.publish(home);

    return true;
}

void getPose(){
    ros::NodeHandle nh;
    //Get home pose
    gobot_msg_srv::GetString get_home;
    ros::service::waitForService("/gobot_status/get_home",ros::Duration(30.0));
    if(ros::service::call("/gobot_status/get_home",get_home)){
        home_pos_x=std::stod(get_home.response.data[0]);
        home_pos_y=std::stod(get_home.response.data[1]);
        home_ang_x=std::stod(get_home.response.data[2]);
        home_ang_y=std::stod(get_home.response.data[3]);
        home_ang_z=std::stod(get_home.response.data[4]);
        home_ang_w=std::stod(get_home.response.data[5]);
        double roll,pitch;
        tf::Matrix3x3(tf::Quaternion(home_ang_x,home_ang_y,home_ang_z,home_ang_w)).getRPY(roll,pitch,home_pos_yaw);
        ROS_INFO("Home: pos(%.2f,%.2f,%.2f),cov(%.2f,%.2f,%.2f).",home_pos_x,home_pos_y,home_pos_yaw,initial_cov_xy,initial_cov_xy,initial_cov_yaw);
    }
    else
        ROS_ERROR("Could not find the param home_pose");
    
    //Get ros server pose
    while(ros::ok()){
        if (ros::param::get("/amcl/initial_pose_x",rosparam_x)){
            ros::param::get("/amcl/initial_pose_y",rosparam_y);
            ros::param::get("/amcl/initial_pose_a",rosparam_yaw);
            ros::param::get("/amcl/initial_cov_xx",rosparam_cov_x);
            ros::param::get("/amcl/initial_cov_yy",rosparam_cov_y);
            ros::param::get("/amcl/initial_cov_aa",rosparam_cov_yaw);
            ROS_INFO("ROS: pos(%.2f,%.2f,%.2f), cov(%.2f,%.2f,%.2f).",rosparam_x,rosparam_y,rosparam_yaw,rosparam_cov_x,rosparam_cov_y,rosparam_cov_yaw);
            break;
        }
        ros::Duration(0.5).sleep();
    }

    //Get last stop pose
    if(nh.hasParam("last_known_position_file")){
        nh.getParam("last_known_position_file", lastPoseFile);
        std::ifstream ifs(lastPoseFile, std::ifstream::in);
        if(ifs){
            ifs >> last_pos_x >> last_pos_y >> last_ang_x >> last_ang_y >> last_ang_z >> last_ang_w;
            ifs.close();
            double roll,pitch;
            tf::Matrix3x3(tf::Quaternion(last_ang_x,last_ang_y,last_ang_z,last_ang_w)).getRPY(roll,pitch,last_pos_yaw);
            ROS_INFO("Last: pos(%.2f,%.2f,%.2f),cov(%.2f,%.2f,%.2f).",last_pos_x,last_pos_y,last_pos_yaw,initial_cov_xy,initial_cov_xy,initial_cov_yaw);
        }
    } 
    else
        ROS_ERROR("Could not find the param last_pose");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "initialpose_estimation");
    ros::NodeHandle nh;
    

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    initial_pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    foundPose_pub = nh.advertise<std_msgs::Int8>("/gobot_recovery/find_initial_pose",10);
    lost_pub = nh.advertise<std_msgs::Int8>("/gobot_recovery/lost_robot",1);

    ros::Subscriber initialPose = nh.subscribe("/amcl_pose",1,getAmclPoseCallback);
    ros::ServiceServer initializeHome = nh.advertiseService("/gobot_recovery/initialize_home",initializeHomeSrcCallback);
    ros::ServiceServer initializePose = nh.advertiseService("/gobot_recovery/initialize_pose",initializePoseSrvCallback);
    ros::ServiceServer globalizePose = nh.advertiseService("/gobot_recovery/globalize_pose",globalizePoseSrvCallback);
    ros::ServiceServer stopGlobalizePose = nh.advertiseService("/gobot_recovery/stop_globalize_pose",stopGlobalizePoseSrvCallback);
    ros::ServiceServer goHome = nh.advertiseService("/gobot_recovery/go_home",goHomeSrvCallback);
    getGobotStatusSrv = nh.serviceClient<gobot_msg_srv::GetGobotStatus>("/gobot_status/get_gobot_status");

    ros::spin();
    return 0;
}
