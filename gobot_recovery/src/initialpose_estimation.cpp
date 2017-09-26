#include <gobot_recovery/initialpose_estimation.h>

double cov_sum=0.0;
double cov_threshold = 0.3, rot_vel = 0.318, rot_time = 23;
bool goalActive = false,globalize_pose = false;
ros::Publisher vel_pub,goalCancel_pub;
std::string lastPoseFile,homeFile;
double last_pos_x=0.0,last_pos_y=0.0,last_ang_x=0.0,last_ang_y=0.0,last_ang_z=0.0,last_ang_w=0.0;

void checkGoalStatus(){
    if(goalActive)
    {
        actionlib_msgs::GoalID arg;
        goalCancel_pub.publish(arg);
        ROS_INFO("Cancelled active goal to proceed gobalo localization. Wait 3 sec to start global localization...");
        ros::Duration(3.0).sleep();
    }
}

bool rotateFindPose(double rot_v,double rot_t){
    checkGoalStatus();
    double dt=0.0;
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z=rot_v;
    //rotate for time rot_t until find small covariance
    ros::Rate loop_rate(5);
    ros::Time last_time = ros::Time::now();
    while(globalize_pose && cov_sum>cov_threshold && dt<rot_t){
        dt=(ros::Time::now() - last_time).toSec();
        ROS_INFO("I am finding my pose in the map...covariance:%.3f",cov_sum);
        vel_pub.publish(vel);
        loop_rate.sleep();
        ros::spinOnce();
    }
    vel.linear.x=0.0;
    vel.angular.z=0.0;
    vel_pub.publish(vel);

    if(cov_sum<cov_threshold)
    {
        ROS_INFO("Found current robot pose in the map. Spent %.3f sec.",dt);
        return true;
    }
    else
    {
        ROS_INFO("Unable to find current robot pose in the map.");
        return false;
    }
}


bool GlobalLocalization(){
    std_srvs::Empty arg;
    if(ros::service::call("/global_localization",arg))
        return rotateFindPose(rot_vel,rot_time*5.0);
    else
        ROS_ERROR("Failed to reset particles.");
}

void publishInitialpose(const double position_x, const double position_y, const double angle_x, const double angle_y, const double angle_z, const double angle_w){

    if(position_x != 0 || position_y != 0 || angle_x != 0 || angle_y != 0 || angle_z != 0 || angle_w != 0){
        ros::NodeHandle n; 

        geometry_msgs::PoseWithCovarianceStamped initialPose;
        initialPose.header.frame_id = "map";
        initialPose.header.stamp = ros::Time::now();
        initialPose.pose.pose.position.x = position_x;
        initialPose.pose.pose.position.y = position_y;
        initialPose.pose.pose.orientation.x = angle_x;
        initialPose.pose.pose.orientation.y = angle_y;
        initialPose.pose.pose.orientation.z = angle_z;
        initialPose.pose.pose.orientation.w = angle_w;
        initialPose.pose.covariance[0] = 0.5;

        ros::Publisher initial_pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        
        // we wait for amcl to launch
        ros::Duration(2.0).sleep();

        initial_pose_publisher.publish(initialPose);

        ROS_INFO("(initial_pose_publisher) initialpose published");
    } else
        ROS_ERROR("(initial_pose_publisher) got a wrong position (robot probably got no home)");
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    cov_sum = 0.0;
    for (int i=0;i<msg->pose.covariance.size();i++){
        cov_sum += std::abs(msg->pose.covariance[i]);
    }
    ROS_INFO("Pose covariance sum:%f",cov_sum);

    //Write lastest amcl_pose to file
    std::ofstream ofs(lastPoseFile, std::ofstream::out | std::ofstream::trunc);
    if(ofs.is_open()){
        ofs << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.orientation.x<<" "<< msg->pose.pose.orientation.y<<" "<< msg->pose.pose.orientation.z<<" "<< msg->pose.pose.orientation.w;
        ofs.close();
    } else
        ROS_ERROR("Could not open the file %s", lastPoseFile.c_str());
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

bool checkInitPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    gobot_msg_srv::IsCharging arg;
    std_srvs::Empty empty_srv;
    globalize_pose = true;
    cov_sum = 100.0;
    ros::service::call("/sensors/isCharging",arg);
    //default pose is charging station 
    //arg.response.isCharging
    if(false){
        //if robot is charging, it is in CS station 
        //clear costmap after finding initial pose
        ros::service::call("/move_base/clear_costmap",empty_srv);
        return true;
    }
    else {
        //check amcl pose covariance by rotating on CS spot
        if(rotateFindPose(rot_vel,rot_time)){
            //robot probably in CS station
            //clear costmap after finding initial pose
            ros::service::call("/move_base/clear_costmap",empty_srv);
            return true;
        }
        else{
            //read last stop pose and initial robot pose
            publishInitialpose(last_pos_x,last_pos_y,last_ang_x,last_ang_y,last_ang_z,last_ang_w);
            ros::Duration(2.0).sleep();
            if(rotateFindPose(rot_vel,rot_time)){
                //robot probably in last stop pose
                //clear costmap after finding initial pose
                ros::service::call("/move_base/clear_costmap",empty_srv);
                return true;
            }
            else
                ros::service::call("/move_base/clear_costmap",empty_srv);
                return GlobalLocalization();
        }
    }

}

bool globalizePoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //check whether have active goal, and cancel it if have
    cov_sum = 100.0;
    globalize_pose = true;
    return GlobalLocalization();
}

bool stopGlobalizePoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    globalize_pose = false;
    return true;
}

void getLastPose(std::string data)
{
    std::string poseStr;
    std::vector<double> pose;
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
        last_pos_x=pose[0];
        last_pos_y=pose[1];
        last_ang_x=pose[2];
        last_ang_y=pose[3];
        last_ang_z=pose[4];
        last_ang_w=pose[5];
        ROS_INFO("Last Pose:position(%.2f,%.2f), orientation(%.2f,%.2f,%.2f,%.2f).",last_pos_x,last_pos_y,last_ang_x,last_ang_y,last_ang_z,last_ang_w);
    }
    else{
         ROS_ERROR("Could not find the last pose infomation");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "initialpose_estimation");
    ros::NodeHandle nh;

    if(nh.hasParam("last_pose_file")){
        nh.getParam("last_pose_file", lastPoseFile);
        ROS_INFO("Set pose file to %s", lastPoseFile.c_str());
        std::ifstream ifs(lastPoseFile, std::ifstream::in);
        if(ifs.is_open()){
            std::string line;
            //Read last pose data from file
            getline(ifs, line);
            //ROS_INFO("Read data:%s", line.c_str());
            ifs.close();
            //Covert pose data from string to double
            getLastPose(line);
        } 
    } else
        ROS_ERROR("Could not find the param last_pose_file");

    if(nh.hasParam("home_file")){
        nh.getParam("home_file", homeFile);
        ROS_INFO("Set home file to %s", homeFile.c_str());
    } else
        ROS_ERROR("Could not find the param home_file");

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    goalCancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);
    ros::Subscriber goalStatus = nh.subscribe("/move_base/status",1,goalStatusCallback);
    ros::Subscriber initialPose = nh.subscribe("/amcl_pose",1,initialPoseCallback);
    ros::ServiceServer checkInitPose = nh.advertiseService("/gobot_recovery/check_init_pose",checkInitPoseCallback);
    ros::ServiceServer globalizePose = nh.advertiseService("/gobot_recovery/globalize_pose",globalizePoseCallback);
    ros::ServiceServer stopGlobalizePose = nh.advertiseService("/gobot_recovery/stop_globalize_pose",stopGlobalizePoseCallback);

    ros::spin();
    return 0;
}
