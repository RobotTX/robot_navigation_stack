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

        if(!bumpers->bumper1)
            cloud.push_back(pcl::PointXYZ(dimension_x, dimension_y, bumpers_height));
        if(!bumpers->bumper2)
            cloud.push_back(pcl::PointXYZ(dimension_x, dimension_y/2, bumpers_height));
        if(!bumpers->bumper3)
            cloud.push_back(pcl::PointXYZ(dimension_x, -dimension_y/2, bumpers_height));
        if(!bumpers->bumper4)
            cloud.push_back(pcl::PointXYZ(dimension_x, -dimension_y, bumpers_height));
        if(!bumpers->bumper5)
            cloud.push_back(pcl::PointXYZ(-dimension_x, -dimension_y, bumpers_height));
        if(!bumpers->bumper6)
            cloud.push_back(pcl::PointXYZ(-dimension_x, -dimension_y/2, bumpers_height));
        if(!bumpers->bumper7)
            cloud.push_back(pcl::PointXYZ(-dimension_x, dimension_y/2, bumpers_height));
        if(!bumpers->bumper8)
            cloud.push_back(pcl::PointXYZ(-dimension_x, dimension_y, bumpers_height));

        pcPublisher.publish(cloud);
    }
}

/// Initialize the global parameters
bool initParams(){
    ros::NodeHandle nh;
    /// We get the frame on which the bumpers are attached
    nh.getParam("pc_frame", pc_frame);
    /// We get the height of the bumpers compared to their frame
    nh.getParam("bumpers_height", bumpers_height);
    nh.getParam("dimension_x", dimension_x);
    nh.getParam("dimension_y", dimension_y);
    nh.getParam("USE_BUMPER_PC", use_pc);
    std::cout <<  "(bumpers2pc::initParams) bumpers height:"<<bumpers_height<<" dimension_x:"<<dimension_x<<" dimension_y:"<<dimension_y<<std::endl;

    return true;
}

int main(int argc, char* argv[]){

    std::cout << "(bumpers2pc) running..." << std::endl;

    try {

        ros::init(argc, argv, "bumpers2pc");

        ros::NodeHandle nh;

        if(initParams()){

            // the pointcloud publisher
            pcPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/gobot_pc/bumpers_pc", 10);

            // get the bumpers collision information
            ros::Subscriber pcSubscriber = nh.subscribe("/gobot_base/bumpers_collision", 1, newBumpersInfo);

            ros::spin();
        }

    } catch (std::exception& e) {
        std::cerr << "(bumpers2pc) Exception: " << e.what() << std::endl;
    }

    return 0;
}