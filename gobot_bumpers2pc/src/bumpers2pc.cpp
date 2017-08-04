#include <gobot_bumpers2pc/bumpers2pc.hpp>

ros::Publisher pcPublisher;
ros::Subscriber statusSuscriber;

struct point_ {
  double x;
  double y;
} ;

std::vector<std::vector<point_>> bumpers_pc;
double space;

/// Convert the bumpers info to a pointcloud
void newBumpersInfo(const gobot_base::BumperMsg::ConstPtr& bumpers){

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "base_link";

    /// 0 : collision; 1 : no collision
    if(!bumpers->bumper1)
        for(int j = 0; j < bumpers_pc.at(0).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(0).at(j).y, -bumpers_pc.at(0).at(j).x, 0));
    if(!bumpers->bumper2)
        for(int j = 0; j < bumpers_pc.at(1).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(1).at(j).y, -bumpers_pc.at(1).at(j).x, 0));
    if(!bumpers->bumper3)
        for(int j = 0; j < bumpers_pc.at(2).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(2).at(j).y, -bumpers_pc.at(2).at(j).x, 0));
    if(!bumpers->bumper4)
        for(int j = 0; j < bumpers_pc.at(3).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(3).at(j).y, -bumpers_pc.at(3).at(j).x, 0));
    if(!bumpers->bumper5)
        for(int j = 0; j < bumpers_pc.at(4).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(4).at(j).y, -bumpers_pc.at(4).at(j).x, 0));
    if(!bumpers->bumper6)
        for(int j = 0; j < bumpers_pc.at(5).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(5).at(j).y, -bumpers_pc.at(5).at(j).x, 0));
    if(!bumpers->bumper7)
        for(int j = 0; j < bumpers_pc.at(6).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(6).at(j).y, -bumpers_pc.at(6).at(j).x, 0));
    if(!bumpers->bumper8)
        for(int j = 0; j < bumpers_pc.at(7).size(); ++j)
            cloud.push_back(pcl::PointXYZ(bumpers_pc.at(7).at(j).y, -bumpers_pc.at(7).at(j).x, 0));

    pcPublisher.publish(cloud);
}

double xToY(const double x, const std::vector<std::vector<double>>& footprint, const bool front_bumper){
    double y = 0;
    if(front_bumper){
        for(int i = 0; i < footprint.size(); ++i){
            if(x >= footprint[i][1] && x <= footprint[(i+1)%footprint.size()][1]){
                double slope = (footprint[(i+1)%footprint.size()][0] - footprint[i][0])/(footprint[(i+1)%footprint.size()][1] - footprint[i][1]);
                double b = footprint[i][0] - slope * footprint[i][1];
                y = slope * x + b;
            }
        }
    } else {
        for(int i = footprint.size() -1; i >= 0; --i){
            if(x >= footprint[i][1] && x <= footprint[(i-1)%footprint.size()][1]){
                double slope = (footprint[(i-1)%footprint.size()][0] - footprint[i][0])/(footprint[(i-1)%footprint.size()][1] - footprint[i][1]);
                double b = footprint[i][0] - slope * footprint[i][1];
                y = slope * x + b;
            }
        }
    }
    return y;
}

/// Initialize the bumper pointcloud which contains the points that will be marked as obstacles when a bumper is pressed
void initBumperPC(const std::vector<std::vector<double>>& footprint, 
    const std::vector<std::vector<double>>& bumpers_description){

    // Bumper pointcloud distance to base frame; should be something like the robot radius plus
    // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
    // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
    // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
    // them will probably fail.

    double min_x(0.0);
    double max_x(0.0);

    for(int i = 0; i < footprint.size(); ++i){
        min_x = std::min(min_x, footprint.at(i).at(1));
        max_x = std::max(max_x, footprint.at(i).at(1));
    }

    std::cout << "(bumpers2pc::initBumperPC) min : " << min_x << " max : " << max_x << std::endl;

    for(int i = 0; i < bumpers_description.size(); ++i){
        std::vector<point_> bumper_pc;
        double x = bumpers_description.at(i).at(0);

        point_ point;
        while(x <= bumpers_description.at(i).at(1) - space && x <= max_x && x >= min_x){
            //std::cout << "(bumpers2pc::initBumperPC) x : " << x << std::endl;
            point.x = x;
            point.y = xToY(x, footprint, i < bumpers_description.size()/2);
            bumper_pc.push_back(point);
            x += space;
        }

        point.x = bumpers_description.at(i).at(1);
        point.y = xToY(bumpers_description.at(i).at(1), footprint, i < bumpers_description.size()/2);
        bumper_pc.push_back(point);

        bumpers_pc.push_back(bumper_pc);
    }
}

/// Initialize the global parameters
bool initParams(void){

    ros::NodeHandle nh("bumpers2pc");

    std::vector<std::vector<double>> footprint;
    std::vector<std::vector<double>> bumpers_description;

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

    if(space <= 0){
        std::cout <<  "(bumpers2pc::initParams) The space between elements should be positive" << std::endl;
        return false;
    }


    std::cout <<  "(bumpers2pc::initParams) space : " << space << std::endl;

    initBumperPC(footprint, bumpers_description);

    for(int32_t i = 0; i < bumpers_pc.size(); ++i){
        std::cout <<  "(bumpers2pc::initParams) Bumper " << i+1 << std::endl;
        for(int32_t j = 0; j < bumpers_pc.at(i).size(); ++j)
            std::cout <<  "(bumpers2pc::initParams) [" << bumpers_pc.at(i).at(j).x << ", " << bumpers_pc.at(i).at(j).y << "]" << std::endl;
    }

    return true;
}

/// Convert a XmlRpcValue variable to a std::vector<std::vector<double>>
bool vectorFromXMLRPC(XmlRpc::XmlRpcValue list, std::vector<std::vector<double>>& vector){

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

        ros::NodeHandle nh("bumpers2pc");

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