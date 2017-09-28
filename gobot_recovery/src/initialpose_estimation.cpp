#include <gobot_recovery/initialpose_estimation.h>


double cov_xy=0.0,cov_yaw=0.0,initial_cov_xy=0.25,initial_cov_yaw=2.0, home_cov_xy=0.15,home_cov_yaw=0.1;
double rot_vel = 0.318, rot_time = 23;
double last_pos_x=0.0,last_pos_y=0.0,last_pos_yaw=0.0,last_ang_x=0.0,last_ang_y=0.0,last_ang_z=0.0,last_ang_w=0.0;
double home_pos_x=0.0,home_pos_y=0.0,home_pos_yaw=0.0,home_ang_x=0.0,home_ang_y=0.0,home_ang_z=0.0,home_ang_w=0.0;
double rosparam_x=0.0,rosparam_y=0.0,rosparam_yaw=0.0,rosparam_cov_x=0.0,rosparam_cov_y=0.0,rosparam_cov_yaw=0.0;

bool goalActive = false;
bool running = false,globalize_pose = false,found_pose=false;;

std_srvs::Empty empty_srv;

ros::Publisher vel_pub,goalCancel_pub,foundPose_pub,initial_pose_publisher;
std::string lastPoseFile,homeFile;

bool evaluatePose(int type){
    //Evaluate ROS server pose 
    if(type==0){
        if((std::abs(rosparam_x)+std::abs(rosparam_y)+std::abs(rosparam_yaw))>1.0 && (std::abs(rosparam_cov_x)+std::abs(rosparam_cov_y)+std::abs(rosparam_cov_yaw))<0.5){
            return true;
        }
        else{
            return false;
        }
    }
    //Evaluate Last pose compared to Home pose
    else if(type==1){
        if(std::abs(home_pos_x-last_pos_x)<1.0 && std::abs(home_pos_y-last_pos_y)<1.0 && std::abs(home_pos_yaw-last_pos_yaw)<PI*15/180){
            return true;
        }
        else{
            return false;
        }
    }
}

void checkGoalStatus(){
    if(goalActive)
    {
        actionlib_msgs::GoalID arg;
        goalCancel_pub.publish(arg);
        ROS_INFO("Cancelled active goal to proceed gobalo localization. Wait 3 sec to start global localization...");
        //wait 3 sec to cancel move_base goal
        ros::Duration(3.0).sleep();
    }
}

bool rotateFindPose(double rot_v,double rot_t){
    checkGoalStatus();
    double dt=0.0;
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z=rot_v;
    //wait 1 sec before rotating
    ros::Duration(1.0).sleep();
    //rotate for time rot_t until find small covariance
    ros::Rate loop_rate(4);
    ros::Time last_time = ros::Time::now();
    //clear costmap before rotating
    ros::service::call("/move_base/clear_costmaps",empty_srv);
    while(globalize_pose && dt<rot_t && (cov_xy>COV_XY_T || cov_yaw>COV_YAW_T)){
        dt=(ros::Time::now() - last_time).toSec();
        //ROS_INFO("I am finding my pose in the map...covariance:xy=%.3f,yaw=%.3f",cov_xy,cov_yaw);
        vel_pub.publish(vel);
        loop_rate.sleep();
    }
    vel.linear.x=0.0;
    vel.angular.z=0.0;
    vel_pub.publish(vel);

    //clear costmap after rotating
    ros::service::call("/move_base/clear_costmaps",empty_srv);

    if(cov_xy<=COV_XY_T && cov_yaw<=COV_YAW_T){
        //Update last pose when found
        found_pose=true;
        ros::service::call("/request_nomotion_update",empty_srv);
        std_msgs::Int8 result;
        result.data = 1;
        //Publish result
        foundPose_pub.publish(result);
        ROS_INFO("Spent %.3f seconds to localize robot pose in the map.",dt);
        return true;
    }
    else{
        return false;
    }
    
}


void GlobalLocalization(){
    if(ros::service::call("/global_localization",empty_srv)){
        if (rotateFindPose(rot_vel,rot_time*5.0)){
            ROS_INFO("Found robot pose after global initialization.");
        }
        else if(globalize_pose){
            ROS_WARN("Unable to find robot pose in the map after global initialization");
            ROS_WARN("Suggest to move robot to charging station and restart");
            //failed to localize
            std_msgs::Int8 result;
            result.data = 0;
            //Publish result
            foundPose_pub.publish(result);
        }
    }
    else
        ROS_ERROR("Failed to reset particles");
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
        ros::Duration(3.0).sleep();

        initial_pose_publisher.publish(initialPose);
        //ROS_INFO("(initial_pose_publisher) initialpose published.");
    } else
        ROS_ERROR("(initial_pose_publisher) got a wrong position (robot probably got no home)");
}

void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if(msg->status_list.empty()) //no goal
        goalActive = false;
    else if (msg->status_list.back().status == 1) //has active goal
        //cancel goal
        goalActive = true;
    else
        goalActive =false;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    cov_xy=std::abs(msg->pose.covariance[0])+std::abs(msg->pose.covariance[7]);
    cov_yaw=std::abs(msg->pose.covariance[35]);
    //ROS_INFO("XY covariance:%.3f,Yaw covariance:%.3f, ",cov_xy,cov_yaw);

    //Write lastest amcl_pose to file
    if(found_pose){
        std::ofstream ofs(lastPoseFile, std::ofstream::out | std::ofstream::trunc);
        if(ofs.is_open()){
            ofs << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.orientation.x<<" "<< msg->pose.pose.orientation.y<<" "<< msg->pose.pose.orientation.z<<" "<< msg->pose.pose.orientation.w;
            ofs.close();
        } 
        else
            ROS_ERROR("Could not open the file %s", lastPoseFile.c_str());
    }
}


bool initializePoseSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(!running){
        std::thread([](){
            int current_state=START_STATE;
            std_msgs::Int8 result;
            gobot_msg_srv::IsCharging arg;
            globalize_pose = true;
            running = true;

            while(current_state!=COMPLETE_STATE && globalize_pose){
                switch(current_state){
                case START_STATE:
                    ROS_INFO("Start finding initial pose...");
                    //start to localize
                    result.data = -1;
                    foundPose_pub.publish(result);
                    current_state=CHARGING_STATE;
                    break;

                case CHARGING_STATE:
                    //wait 3 second for batter status update
                    ros::Duration(3.0).sleep();
                    ros::service::call("/isCharging",arg);
                    //if robot is charging, it is in CS station 
                    if(arg.response.isCharging||evaluatePose(1)){
                    //if(false){
                        found_pose=true;
                        ROS_INFO("Robot is charging in the charing station");
                        publishInitialpose(home_pos_x,home_pos_y,home_ang_x,home_ang_y,home_ang_z,home_ang_w,home_cov_xy,home_cov_yaw);
                        result.data = 1;
                        foundPose_pub.publish(result);
                        current_state=COMPLETE_STATE;
                    }
                    else{
                        ROS_WARN("Robot is not charging");
                        current_state=ROSPARAM_POSE_STATE;
                    }
                    break;

                case ROSPARAM_POSE_STATE:
                    ROS_INFO("Try rosparam server pose...");
                    if(evaluatePose(0)){
                        //try the pose from rosparam server
                        if(rotateFindPose(rot_vel,rot_time)){
                            ROS_INFO("Robot is near the rosparam server pose.");
                            current_state=COMPLETE_STATE;
                        }
                        else{
                            ROS_WARN("Robot is not in the rosparam server pose");
                            //try the pose from last stop pose
                            current_state=LAST_POSE_STATE;
                        }
                    }
                    else{
                        ROS_WARN("rosparam server pose is not reliable");
                        current_state=LAST_POSE_STATE;
                    }
                    break;

                case LAST_POSE_STATE:
                    //try the pose from last stop pose
                    ROS_INFO("Try last stop pose...");
                    publishInitialpose(last_pos_x,last_pos_y,last_ang_x,last_ang_y,last_ang_z,last_ang_w,initial_cov_xy,initial_cov_yaw);
                    if(rotateFindPose(rot_vel,rot_time)){
                        ROS_INFO("Robot is near the last stop pose.");
                        current_state=COMPLETE_STATE;
                    }
                    else {
                        ROS_WARN("Robot is not in the last stop pose");
                        //try the pose from home pose
                        current_state=GLOBAL_POSE_STATE;
                    }
                    break;

                case GLOBAL_POSE_STATE:
                    //try globalize pose after all recorded pose failed
                    ROS_INFO("Try reset particles...");
                    GlobalLocalization();
                    current_state=COMPLETE_STATE;
                    break;

                default:
                    current_state=COMPLETE_STATE;
                    break;
                }
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
            int current_state=START_STATE;
            std_msgs::Int8 result;
            globalize_pose = true;
            running = true;

            while(current_state!=COMPLETE_STATE && globalize_pose){
                switch(current_state){
                case START_STATE:
                    ROS_INFO("Start globalizing pose...");
                    //start to localize
                    result.data = -1;
                    foundPose_pub.publish(result);
                    current_state=GLOBAL_POSE_STATE;
                    break;

                case GLOBAL_POSE_STATE:
                    GlobalLocalization();
                    current_state=COMPLETE_STATE;
                    break;

                default:
                    current_state=COMPLETE_STATE;
                    break;
                }
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
        globalize_pose = false;
        ROS_INFO("Stop finding initial pose.");
         //cancel localization
        std_msgs::Int8 result;
        result.data = 2;
        //Publish result
        foundPose_pub.publish(result);
    }
    else{
        ROS_WARN("Robot is not finding initial pose");
    }
    return true;
}

//type=0=home_pose,type=1=last_pose
void getPose(std::string file_name,int type)
{
    std::string poseStr;
    std::vector<double> pose;
    std::ifstream ifs(file_name, std::ifstream::in);
    if(ifs.is_open()){
    std::string data;
    //Read last pose data from file
    getline(ifs, data);
    //ROS_INFO("Read data:%s", line.c_str());

    for(int i=0;i<data.length();i++){
        if(data[i]==' '){
            if(!poseStr.empty()){
                pose.push_back(std::atof(poseStr.c_str()));
                poseStr.clear();
            }
        }
        else{
            poseStr.push_back(data[i]);
        }
    }
    pose.push_back(std::atof(poseStr.c_str()));

    if(pose.size()==6){
        if(type==1)
        {
            last_pos_x=pose[0];
            last_pos_y=pose[1];
            last_ang_x=pose[2];
            last_ang_y=pose[3];
            last_ang_z=pose[4];
            last_ang_w=pose[5];
            double roll,pitch;
            tf::Matrix3x3(tf::Quaternion(pose[2],pose[3],pose[4],pose[5])).getRPY(roll,pitch,last_pos_yaw);
            ROS_INFO("Last: pos(%.2f,%.2f,%.2f),cov(%.2f,%.2f,%.2f).",last_pos_x,last_pos_y,last_pos_yaw,initial_cov_xy,initial_cov_xy,initial_cov_yaw);
        }
        else if(type==0)
        {
            home_pos_x=pose[0];
            home_pos_y=pose[1];
            home_ang_x=pose[2];
            home_ang_y=pose[3];
            home_ang_z=pose[4];
            home_ang_w=pose[5];
            double roll,pitch;
            tf::Matrix3x3(tf::Quaternion(pose[2],pose[3],pose[4],pose[5])).getRPY(roll,pitch,home_pos_yaw);
            ROS_INFO("Home: pos(%.2f,%.2f,%.2f),cov(%.2f,%.2f,%.2f).",home_pos_x,home_pos_y,home_pos_yaw,home_cov_xy,home_cov_xy,home_cov_yaw);
        }
    }
    else{
        ROS_ERROR("Wrong pose infomation in the file");
    }
    ifs.close();
    //Covert pose data from string to double
    } 
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "initialpose_estimation");
    ros::NodeHandle nh;

    if(nh.hasParam("home_pose_file")){
        nh.getParam("home_pose_file", homeFile);
        //Get charging station pose
        getPose(homeFile,0);
    } 
    else
        ROS_ERROR("Could not find the param home_pose_file");

    if(nh.hasParam("last_pose_file")){
        nh.getParam("last_pose_file", lastPoseFile);
        //Get last stop pose
        getPose(lastPoseFile,1);
    } 
    else
        ROS_ERROR("Could not find the param last_pose_file");
    

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    goalCancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);
    foundPose_pub = nh.advertise<std_msgs::Int8>("/gobot_recovery/find_initial_pose",10);
    initial_pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Subscriber goalStatus = nh.subscribe("/move_base/status",1,goalStatusCallback);
    ros::Subscriber initialPose = nh.subscribe("/amcl_pose",1,initialPoseCallback);
    ros::ServiceServer initializePose = nh.advertiseService("/gobot_recovery/initialize_pose",initializePoseSrvCallback);
    ros::ServiceServer globalizePose = nh.advertiseService("/gobot_recovery/globalize_pose",globalizePoseSrvCallback);
    ros::ServiceServer stopGlobalizePose = nh.advertiseService("/gobot_recovery/stop_globalize_pose",stopGlobalizePoseSrvCallback);

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
        else{
            ROS_WARN("Could not get the param from ROS server");
        }
        ros::Duration(0.5).sleep();
    }


    ros::spin();
    return 0;
}
