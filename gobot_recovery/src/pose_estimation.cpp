#include <gobot_recovery/pose_estimation.h>


double cov_x=0.0,cov_y=0.0,cov_yaw=0.0,initial_cov_xy=0.02,initial_cov_yaw=0.02;
//Make robot rotate 360 degrees
double rot_vel = 0.4, rot_time = 150;
double last_pos_x=0.0,last_pos_y=0.0,last_ang_x=0.0,last_ang_y=0.0,last_ang_z=0.0,last_ang_w=0.0;
double home_pos_x=0.0,home_pos_y=0.0,home_ang_x=0.0,home_ang_y=0.0,home_ang_z=0.0,home_ang_w=0.0;
double rosparam_x=0.0,rosparam_y=0.0,rosparam_yaw=0.0,rosparam_cov_x=0.0,rosparam_cov_y=0.0,rosparam_cov_yaw=0.0;

bool running = false,found_pose=false, charging_flag_ = false;

std_srvs::Empty empty_srv;

int left_speed_ = 0, right_speed_ = 0;

double UPDATE_DURATION = 5.0;
int UPDATE_NUM = 20, update_count = 0;

ros::Publisher vel_pub, foundPose_pub, goal_pub,lost_pub;
std::string lastPoseFile;
ros::ServiceServer poseReadySrv;
ros::Timer pose_timer;
ros::Time zero_vel_time;

robot_class::SetRobot SetRobot;
robot_class::GetRobot GetRobot;
int robot_status_;
std::string status_text_;

bool poseReadySrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    return true;
}

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
            if(fabs(home_pos_x-last_pos_x)<0.1 && fabs(home_pos_y-last_pos_y)<0.1){
                ROS_INFO("(POSE_ESTIMATION) Last recorded post near to charging station.");
                return true;
            }
        return false;
    }
    return false;
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
        //ROS_INFO("(POSE_ESTIMATION) I am finding my pose in the map...covariance:xy=%.3f,yaw=%.3f",cov_xy,cov_yaw);
        vel_pub.publish(vel);
        if(cov_x<COV_XY_T && cov_y<COV_XY_T && cov_yaw<COV_YAW_T){
            ROS_INFO("(POSE_ESTIMATION) Spent %.3f seconds to localize robot pose in the map.",dt);
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

bool initializePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(!running){
        std::thread([](){
            int current_stage=START_STAGE;
            running = true;
            found_pose=false;

            int move_status = SetRobot.stopRobotMoving();
            if(move_status != 0){
                ROS_INFO("(POSE_ESTIMATION) Stop robot motion, current status: %d.", move_status);
            }

            //get pose
            getPose();

            while(current_stage!=COMPLETE_STAGE && running && ros::ok()){
                switch(current_stage){
                    case START_STAGE:
                        ROS_INFO("(POSE_ESTIMATION) Start finding initial pose...");
                        //start to localize
                        findPoseResult(START_FOUND);
                        current_stage=CHARGING_STAGE;
                        break;

                    case CHARGING_STAGE:
                        //if robot is charging, it is in CS station 
                        if(charging_flag_){
                            SetRobot.setInitialpose(home_pos_x,home_pos_y,home_ang_x,home_ang_y,home_ang_z,home_ang_w);
                            ROS_WARN("(POSE_ESTIMATION) Robot is in the charing station");
                            found_pose=true;
                            current_stage=COMPLETE_STAGE;
                        }
                        else{
                            current_stage=ROSPARAM_POSE_STAGE;
                        }
                        break;

                    case ROSPARAM_POSE_STAGE:
                        if(evaluatePose(0)){
                            ROS_WARN("(POSE_ESTIMATION) Robot is near the rosparam server pose.");
                            found_pose=true;
                            current_stage=COMPLETE_STAGE;
                        }
                        else{
                            current_stage=LAST_POSE_STAGE;
                        }
                        break;

                    case LAST_POSE_STAGE:
                        //try the pose from last stop pose
                        SetRobot.setInitialpose(last_pos_x,last_pos_y,last_ang_x,last_ang_y,last_ang_z,last_ang_w);
                        ROS_WARN("(POSE_ESTIMATION) Robot is near the last stop pose.");
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
                ros::NodeHandle nh;
                SetRobot.setStatus(-2,"STARTUP_READY");
                poseReadySrv = nh.advertiseService("/gobot_startup/pose_ready", poseReadySrvCallback);
                SetRobot.setBatteryLed();
                //Startup end

                zero_vel_time = ros::Time::now();
                update_count = UPDATE_NUM;
                pose_timer.start();
            }
            running = false;
        }).detach();
        return true;
    }
    else{
        ROS_ERROR("(POSE_ESTIMATION) Localization is running by other request and not complete yet. Call /gobot_recovery/stop_globalize_pose service to stop it");
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
            
            int move_status = SetRobot.stopRobotMoving();
            if(move_status != 0){
                ROS_INFO("(POSE_ESTIMATION) Stop robot motion, current status: %d.", move_status);
            }
            
            while(current_stage!=COMPLETE_STAGE && running && ros::ok()){
                switch(current_stage){
                    case START_STAGE:
                        ROS_INFO("(POSE_ESTIMATION) Start globalizing pose...");
                        //start to localize
                        findPoseResult(START_FOUND);
                        current_stage=GLOBAL_POSE_STAGE;
                        break;

                    case GLOBAL_POSE_STAGE:
                        if(ros::service::call("/global_localization",empty_srv)){
                            ros::Duration(3.0).sleep();
                            found_pose=rotateFindPose(rot_vel,rot_time);
                            if (found_pose){
                                ROS_INFO("(POSE_ESTIMATION) Found robot pose after global initialization.");
                            }
                            else if(running){
                                ROS_WARN("(POSE_ESTIMATION) Unable to find robot pose in the map after global initialization");
                                ROS_WARN("(POSE_ESTIMATION) Suggest to move robot to charging station and restart");
                                //failed to localize
                                findPoseResult(NOT_FOUND);
                            }
                        }
                        else{
                            ROS_ERROR("(POSE_ESTIMATION) Failed to reset particles");
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
        ROS_ERROR("(POSE_ESTIMATION) Localization is running by other request and not complete yet. Call /gobot_recovery/stop_globalize_pose service to stop it");
        return false;
    }
}

bool stopGlobalizePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(running){
        running = false;
        ROS_INFO("(POSE_ESTIMATION) Stop finding initial pose.");
        //cancel localization
        if(!found_pose)
            findPoseResult(CANCEL_FOUND);
    }
    else{
        ROS_WARN("(POSE_ESTIMATION) Robot is not finding initial pose");
    }
    return true;
}

void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    /*
    cov_x=msg->pose.covariance[0];
    cov_y=msg->pose.covariance[7];
    cov_yaw=msg->pose.covariance[35];

    //ROS_INFO("(POSE_ESTIMATION) cov_x:%.4f,cov_y:%.4f,cov_yaw:%.4f",cov_x,cov_y,cov_yaw);
    //Write lastest amcl_pose to file
    if(found_pose){
        std_msgs::Int8 lost;
        if((cov_x > 5 && cov_y > 5) || cov_yaw > 1){
            if(cov_yaw > 1)
                ROS_ERROR("(POSE_ESTIMATION) Big yaw covariance in the amcl pose");
            if(cov_x > 5 && cov_y > 5)
                ROS_ERROR("(POSE_ESTIMATION) Big xy covariance in the amcl pose");
            lost.data = 1; 
        }
        else{
            lost.data=0;
        }
        lost_pub.publish(lost);
    }
    */
}

bool goHomeSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //get pose
    getPose();

    GetRobot.getStatus(robot_status_);
    //pause when goal active or goal reached(may wait for human action)
    int move_status = SetRobot.stopRobotMoving();
    if(move_status != 0){
        ROS_INFO("(POSE_ESTIMATION) Stop robot motion, current status: %d.", move_status);
    }

    ROS_INFO("(POSE_ESTIMATION) Go Home, sweet home.");
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
    GetRobot.getHome(home_pos_x,home_pos_y,home_ang_x,home_ang_y,home_ang_z,home_ang_w);
    ROS_INFO("(POSE_ESTIMATION) HOME_POSE: pos(%.2f,%.2f),cov(%.2f,%.2f,%.2f).",home_pos_x,home_pos_y,initial_cov_xy,initial_cov_xy,initial_cov_yaw);

    
    //Get ros server pose
    while(ros::ok()){
        if (ros::param::get("/amcl/initial_pose_x",rosparam_x)){
            ros::param::get("/amcl/initial_pose_y",rosparam_y);
            ros::param::get("/amcl/initial_pose_a",rosparam_yaw);
            ros::param::get("/amcl/initial_cov_xx",rosparam_cov_x);
            ros::param::get("/amcl/initial_cov_yy",rosparam_cov_y);
            ros::param::get("/amcl/initial_cov_aa",rosparam_cov_yaw);
            ROS_INFO("(POSE_ESTIMATION) ROS POSE: pos(%.2f,%.2f,%.2f), cov(%.2f,%.2f,%.2f).",rosparam_x,rosparam_y,rosparam_yaw,rosparam_cov_x,rosparam_cov_y,rosparam_cov_yaw);
            break;
        }
        ros::Duration(0.5).sleep();
    }

    //Get last stop pose
    if(nh.hasParam("last_pose_file")){
        nh.getParam("last_pose_file", lastPoseFile);
        std::ifstream ifs(lastPoseFile, std::ifstream::in);
        if(ifs){
            ifs >> last_pos_x >> last_pos_y >> last_ang_x >> last_ang_y >> last_ang_z >> last_ang_w;
            ifs.close();
            double roll,pitch;
            ROS_INFO("(POSE_ESTIMATION) Last POSE: pos(%.2f,%.2f),cov(%.2f,%.2f,%.2f).",last_pos_x,last_pos_y,initial_cov_xy,initial_cov_xy,initial_cov_yaw);
        }
    } 
    else
        ROS_ERROR("(POSE_ESTIMATION) Could not find the param last_pose");
}


void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg){
    charging_flag_ = msg->ChargingFlag;
}

void motorSpdCallback(const gobot_msg_srv::MotorSpeedMsg::ConstPtr& speed){
    left_speed_ = speed->velocityL;
    right_speed_ = speed->velocityR;
    if(left_speed_!=0 || right_speed_!=0){
        zero_vel_time = ros::Time::now();
        update_count = 0;
    }
}


//manually perform update and publish updated particles
void UpdateRobotPosTimer(const ros::TimerEvent&){
    if(ros::service::exists("/request_nomotion_update",false)){
        //if robot stopped for more than x sec and not charging, we start to update pose particles
        if((ros::Time::now()-zero_vel_time)>ros::Duration(UPDATE_DURATION) && !charging_flag_){
            if(left_speed_==0 && right_speed_==0 && update_count<UPDATE_NUM){
                ros::service::call("/request_nomotion_update",empty_srv);
                update_count++;
            }
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle nh;
    SetRobot.initialize();
    
    nh.getParam("UPDATE_DURATION", UPDATE_DURATION);
    nh.getParam("UPDATE_NUM", UPDATE_NUM);
    
    vel_pub = nh.advertise<geometry_msgs::Twist>("/teleop_cmd_vel",5);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    foundPose_pub = nh.advertise<std_msgs::Int8>("/gobot_recovery/find_initial_pose",1);
    lost_pub = nh.advertise<std_msgs::Int8>("/gobot_recovery/lost_robot",1);

    ros::ServiceServer initializePose = nh.advertiseService("/gobot_recovery/initialize_pose",initializePoseSrvCallback);
    ros::ServiceServer globalizePose = nh.advertiseService("/gobot_recovery/globalize_pose",globalizePoseSrvCallback);
    ros::ServiceServer stopGlobalizePose = nh.advertiseService("/gobot_recovery/stop_globalize_pose",stopGlobalizePoseSrvCallback);
    ros::ServiceServer goHome = nh.advertiseService("/gobot_recovery/go_home",goHomeSrvCallback);

    ros::Subscriber motorSpd_sub = nh.subscribe("/gobot_motor/motor_speed", 1, motorSpdCallback);
    ros::Subscriber initialPose_sub = nh.subscribe("/amcl_pose",1,getAmclPoseCallback);
    ros::Subscriber battery_sub = nh.subscribe("/gobot_base/battery_topic",1, batteryCallback);

    //Periodically update robot pose in the map by amcl
    pose_timer = nh.createTimer(ros::Duration(UPDATE_DURATION), UpdateRobotPosTimer);
    pose_timer.stop();

    ros::spin();
    return 0;
}
