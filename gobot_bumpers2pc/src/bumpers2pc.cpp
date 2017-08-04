#include <gobot_bumpers2pc/bumpers2pc.hpp>

ros::Publisher pcPublisher;
ros::Subscriber statusSuscriber;

std::vector<std::vector<double>> footprint;
std::vector<std::vector<double>> bumpers_description;
double space;

/// Convert the bumpers info to a pointcloud
void newBumpersInfo(const gobot_base::BumperMsg::ConstPtr& bumpers){

    /// 0 : collision; 1 : no collision
    bool front = !(bumpers->bumper1 && bumpers->bumper2 && bumpers->bumper3 && bumpers->bumper4);
    bool back = !(bumpers->bumper5 && bumpers->bumper6 && bumpers->bumper7 && bumpers->bumper8);

    //std::cout << "(bumpers2pc::newBumpersInfo) Bumpers: " << bumpers->bumper1 << " " << bumpers->bumper2 << " " << bumpers->bumper3 << " " << bumpers->bumper4 << " " << bumpers->bumper5 << " " << bumpers->bumper6 << " " << bumpers->bumper7 << " " << bumpers->bumper8 << std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Bumper pointcloud distance to base frame; should be something like the robot radius plus
    // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
    // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
    // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
    // them will probably fail.


}

/// Initialize the global parameters
bool initParams(void){

    ros::NodeHandle nh;


    /// We get the footprint as a XmlRpcValue
    XmlRpc::XmlRpcValue footprint_list;
    if(!nh.getParam("footprint", footprint_list)){
        std::cout << "(bumpers2pc::initParams) Failed to get parameter \"footprint\" from server." << std::endl;
        return false;
    }

    /// Convert the footprint from a XmlRpcValue to a vector of double
    if(!vectorFromXMLRPC(footprint_list, footprint))
        return false;

    /// Verify the validity of the footprint
    std::cout <<  "(bumpers2pc::initParams) footprint size : " << footprint.size() << std::endl;
    for(int32_t i = 0; i < footprint.size(); ++i){
        if(footprint.at(i).size() != 2){
            std::cout <<  "(bumpers2pc::initParams) The element " << i << " of the footprint does not contain 2 numbers but " << footprint.at(i).size() << std::endl;
            return false;
        }
        std::cout <<  "(bumpers2pc::initParams) [" << footprint.at(i).at(0) << ", " << footprint.at(i).at(1) << "]" << std::endl;
    }


    /// We get the bumpers description as a XmlRpcValue
    XmlRpc::XmlRpcValue bumpers_description_list;
    if(!nh.getParam("bumpers_description", bumpers_description_list)){
        std::cout <<  "(bumpers2pc::initParams) Failed to get parameter \"bumpers_description\" from server." << std::endl;
        return false;
    }

    /// Convert the bumpers description from a XmlRpcValue to a vector of double
    if(!vectorFromXMLRPC(bumpers_description_list, bumpers_description) || bumpers_description.size() != 8)
        return false;

    /// Verify the validity of the bumpers description
    std::cout <<  "(bumpers2pc::initParams) bumpers_description size :" << bumpers_description.size() << std::endl;
    if(bumpers_description.size() != 8){
        std::cout <<  "(bumpers2pc::initParams) The bumpers_description is supposed to be made of 8 bumpers, not " << bumpers_description.size() << std::endl;
        return false;
    }

    for(int32_t i = 0; i < bumpers_description.size(); ++i){
        if(bumpers_description.at(i).size() != 2){
            std::cout <<  "(bumpers2pc::initParams) The element " << i << " of the bumpers_description does not contain 2 numbers but " << bumpers_description.at(i).size() << std::endl;
            return false;
        }
        std::cout <<  "(bumpers2pc::initParams) [" << bumpers_description.at(i).at(0) << ", " << bumpers_description.at(i).at(1) << "]" << std::endl;
    }


    /// We get the space between each point in the cloud
    nh.param("space", space, 0.05);

    std::cout <<  "(bumpers2pc::initParams) space " << space << std::endl;

    return true;
}

/// Convert a XmlRpcValue variable to a std::vector<std::vector<double>>
bool vectorFromXMLRPC(XmlRpc::XmlRpcValue list, std::vector<std::vector<double>> &vector){

    /// Check that we have an array (or a list)
    if(list.getType() != XmlRpc::XmlRpcValue::TypeArray){
        std::cout <<  "(bumpers2pc::vectorFromXMLRPC) The list is not an array, type : " << list.getType() << std::endl;
        return false;
    }

    for(int32_t i = 0; i < list.size(); ++i){
        /// Check that our array is filled with another array
        if(list[i].getType() != XmlRpc::XmlRpcValue::TypeArray){
            std::cout <<  "(bumpers2pc::vectorFromXMLRPC) The inner list is not an array, type : " << list[i].getType() << std::endl;
            return false;
        }

        std::vector<double> point;

        for (int32_t j = 0; j < list[i].size(); ++j){
            /// Check that we got a number (double or int)
            if(list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble && list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt){
                std::cout <<  "(bumpers2pc::vectorFromXMLRPC) The inner element is not a double or int, type : " << list[i][j].getType() << std::endl;
                return false;
            }

            if(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                point.push_back(static_cast<double>(list[i][j]));
            else if(list[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
                point.push_back(static_cast<int>(list[i][j]));
        }

        vector.push_back(point);
    }

    return true;
}

int main(int argc, char* argv[]){

    std::cout << "(bumpers2pc) running..." << std::endl;

    try {

        ros::init(argc, argv, "bumpers2pc");

        ros::NodeHandle nh("~/bumpers2pc");

        if(initParams()){

            // publish the pointcloud
            pcPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/bumpers_pc", 10);

            // get the bumpers information
            statusSuscriber = nh.subscribe("/bumpers_topic", 1, newBumpersInfo);

            ros::spin();
        }

    } catch (std::exception& e) {
        std::cerr << "(bumpers2pc) Exception: " << e.what() << std::endl;
    }

    return 0;
}