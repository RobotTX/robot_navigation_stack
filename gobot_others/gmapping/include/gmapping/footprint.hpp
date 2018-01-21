#ifndef FOOTPRINT
#define FOOTPRINT

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <string>
#include <vector>
#include <nav_msgs/GetMap.h>
#include <XmlRpcValue.h>
#include <unordered_set>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

typedef std::pair<double, double> pairs;

class Footprint {
    public:
        Footprint();
        bool init(const XmlRpc::XmlRpcValue xml_footprint);
        bool updateFootprint(nav_msgs::GetMap::Response& map, const std::string map_frame, const ros::Time stamp, const std::string base_frame);
        void addFootprintToMap(nav_msgs::GetMap::Response& map);

    private:
        bool vectorFromXMLRPC(XmlRpc::XmlRpcValue list, std::vector<std::vector<double>>& vector);
        std::vector<double> getXVector(const std::vector<std::vector<double>> transformed_footprint, const double y);

    private:
        std::vector<std::vector<double>> footprint;
        tf::TransformListener tf;
        std::set<pairs> filled_footprint_set;
};

#endif