#include <gmapping/footprint.hpp>

Footprint::Footprint(){}

bool Footprint::init(const XmlRpc::XmlRpcValue xml_footprint){

    /// Convert the footprint from a XmlRpcValue to a vector of double
    if(!vectorFromXMLRPC(xml_footprint, footprint)){
        ROS_ERROR("The footprint could not be converted into a vector");
        return false;
    }

    /// Verify the validity of the footprint
    ROS_INFO("(Footprint::init) footprint size : %lu", footprint.size());
    for(int32_t i = 0; i < footprint.size(); ++i){
        if(footprint.at(i).size() != 2){
            ROS_INFO("(Footprint::init) The element %d of the footprint does not contain 2 numbers but %lu", i, footprint.at(i).size());
            return false;
        }
    }

    return true;
}

/// Convert a XmlRpcValue variable to a std::vector<std::vector<double>>
bool Footprint::vectorFromXMLRPC(XmlRpc::XmlRpcValue list, std::vector<std::vector<double>>& vector){

    /// Check that we have an array (or a list)
    if(list.getType() != XmlRpc::XmlRpcValue::TypeArray){
        ROS_INFO("(Footprint::vectorFromXMLRPC) The list is not an array, type : %d", list.getType());
        return false;
    }

    for(int32_t i = 0; i < list.size(); ++i){
        /// Check that our array is filled with another array
        if(list[i].getType() != XmlRpc::XmlRpcValue::TypeArray){
            ROS_INFO("(Footprint::vectorFromXMLRPC) The inner list is not an array, type : %d", list[i].getType());
            return false;
        }

        std::vector<double> point;

        for (int32_t j = 0; j < list[i].size(); ++j){
            /// Check that we got a number (double or int)
            if(list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble && list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt){
                ROS_INFO("(Footprint::vectorFromXMLRPC) The inner element is not a double or int, type : %d", list[i][j].getType());
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


bool Footprint::updateFootprint(nav_msgs::GetMap::Response& map, const std::string map_frame, const ros::Time stamp, const std::string base_frame){

    /// We want to tranform our footprint polygon from the base_frame to the map_frame
    std::vector<std::vector<double>> transformed_footprint;
    for(int32_t i = 0; i < footprint.size(); ++i){
        /// Create a stamped point32_t of the footprint edge in the base_frame
        tf::Stamped<tf::Vector3> stamped_point(tf::Vector3(footprint.at(i).at(0), footprint.at(i).at(1), 0), stamp, base_frame);
        tf::Stamped<tf::Vector3> stamped_out;
        try {
            /// Transform the footprint edge from the base_frame to the map_frame
            tf.waitForTransform(map_frame, base_frame, ros::Time(0), ros::Duration(1.0) );
            tf.transformPoint(map_frame, stamped_point, stamped_out);
            //ROS_INFO("Stamped from [%f, %f] to [%f, %f]", stamped_point.x(), stamped_point.y(), stamped_out.x(), stamped_out.y());
            std::vector<double> v;
            v.push_back(stamped_out.x());
            v.push_back(stamped_out.y());
            transformed_footprint.push_back(v);
        } catch(tf::TransformException& e) {
            ROS_WARN("Unable to tranform : %s", e.what());
            return false;
        }
    }

    ROS_INFO("transformed_footprint size : %lu", transformed_footprint.size());

    /// Scan fill algorithm
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for(int32_t i = 0; i < transformed_footprint.size(); ++i){
        minX = transformed_footprint.at(i).at(0) < minX ? transformed_footprint.at(i).at(0) : minX;
        minY = transformed_footprint.at(i).at(1) < minY ? transformed_footprint.at(i).at(1) : minY;
        maxX = transformed_footprint.at(i).at(0) > maxX ? transformed_footprint.at(i).at(0) : maxX;
        maxY = transformed_footprint.at(i).at(1) > maxY ? transformed_footprint.at(i).at(1) : maxY;
    }

    float y = floor((minY + map.map.info.resolution/(double)2)*100+0.5)/(double)100; //floor(num*100+0.5)/100;
    while(y <= maxY) {
        std::vector<double> x_vector = getXVector(transformed_footprint, y);

        if(x_vector.size() >= 2 && x_vector.size() % 2 == 0){
            std::sort(x_vector.begin(), x_vector.end());
            for(int32_t i = 0; i < x_vector.size(); i += 2){
                float x = floor((x_vector.at(i) + map.map.info.resolution/(double)2)*100+0.5)/(double)100;
                while(x <= x_vector.at(i+1)) {
                    filled_footprint_set.insert(pairs(x, y));
                    x += map.map.info.resolution;
                }
            }
        } //else
            //ROS_INFO("Something is wrong with the x_vector size : %lu; x : %f; y : %f", x_vector.size(), x_vector.size() > 0 ? x_vector.at(0) : -20, y);

        y += map.map.info.resolution;
    }

    return true;
}

void Footprint::addFootprintToMap(nav_msgs::GetMap::Response& map){
    ROS_INFO("filled_footprint_set size : %lu", filled_footprint_set.size());

    for (auto i = filled_footprint_set.begin(); i != filled_footprint_set.end(); ++i)
        map.map.data[(int) MAP_IDX(map.map.info.width, 
            (unsigned int)((i->first - map.map.info.origin.position.x)/map.map.info.resolution), 
            (unsigned int)((i->second - map.map.info.origin.position.y)/map.map.info.resolution))] = 0;
}

std::vector<double> Footprint::getXVector(const std::vector<std::vector<double>> transformed_footprint, const double y){
    std::vector<double> x_vector;
    for(int32_t i = 0; i < transformed_footprint.size(); ++i){
        /// Get intersection of the polygon on the given y
        std::vector<double> pt1 = transformed_footprint.at(i);
        std::vector<double> pt2 = transformed_footprint.at((i+1) % transformed_footprint.size());
        if(y >= pt1[1] && y <= pt2[1]){
            //ROS_INFO("%f is between %f and %f", y, pt1[1], pt2[1]);
            double slope = (pt2[0] - pt1[0])/(double)(pt2[1] - pt1[1]);
            double b = pt1[0] - slope * pt1[1];
            x_vector.push_back(slope * y + b);
        } else if(y <= pt1[1] && y >= pt2[1]){
            //ROS_INFO("%f is between %f and %f", y, pt1[1], pt2[1]);
            double slope = (pt1[0] - pt2[0])/(double)(pt1[1] - pt2[1]);
            double b = pt2[0] - slope * pt2[1];
            x_vector.push_back(slope * y + b);
        }
    }

    return x_vector;
}