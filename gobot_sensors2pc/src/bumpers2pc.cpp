#include <gobot_sensors2pc/bumpers2pc.hpp>

ros::Publisher pcPublisher;

double space, bumpers_height,dimension_x, dimension_y;
bool use_pc = true;
std::string pc_frame;

/// Convert the bumpers info to a pointcloud and publish it
void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    if(use_pc){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.header.frame_id = pc_frame;

        if(!bumpers->bumper1 || !bumpers->bumper2){
            cloud.push_back(pcl::PointXYZ(dimension_x, dimension_y, bumpers_height));
            cloud.push_back(pcl::PointXYZ(dimension_x, dimension_y/2, bumpers_height));
        }
        if(!bumpers->bumper3 || !bumpers->bumper4){
            cloud.push_back(pcl::PointXYZ(dimension_x, -dimension_y/2, bumpers_height));
            cloud.push_back(pcl::PointXYZ(dimension_x, -dimension_y, bumpers_height));
        }
        if(!bumpers->bumper5 || !bumpers->bumper6){
            cloud.push_back(pcl::PointXYZ(-dimension_x, -dimension_y, bumpers_height));
            cloud.push_back(pcl::PointXYZ(-dimension_x, -dimension_y/2, bumpers_height));
        }
        if(!bumpers->bumper7 || !bumpers->bumper8){
            cloud.push_back(pcl::PointXYZ(-dimension_x, dimension_y/2, bumpers_height));
            cloud.push_back(pcl::PointXYZ(-dimension_x, dimension_y, bumpers_height));
        }

        pcPublisher.publish(cloud);
    }
}

/// Initialize the global parameters
void initParams(){
    ros::NodeHandle nh;
    /// We get the frame on which the bumpers are attached
    nh.getParam("PC_FRAME", pc_frame);
    /// We get the height of the bumpers compared to their frame
    nh.getParam("DIM_X", dimension_x);
    nh.getParam("DIM_Y", dimension_y);
    nh.getParam("USE_BUMPER_PC", use_pc);
    nh.getParam("DIM_Z", bumpers_height);
    std::cout <<  "(BUMPER2PC::initParams) bumpers height:"<<bumpers_height<<" dimension_x:"<<dimension_x<<" dimension_y:"<<dimension_y<<std::endl;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "bumpers2pc");
    ros::NodeHandle nh;
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    //Startup end

    initParams();

    // the pointcloud publisher
    pcPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/bumpers_pc", 10);

    // get the bumpers collision information
    ros::Subscriber pcSubscriber = nh.subscribe("/gobot_base/bumpers_collision", 1, newBumpersInfo);

    ros::spin();

    return 0;
}