#ifndef CLUSTER_OPTIMIZED_H
#define CLUSTER_OPTIMIZED_H

#include "ros/ros.h"
#include <geometry_msgs/Pose.h> 
#include <vector>
#include <math.h> 

class Cluster
{
public:
	Cluster(const std::vector<geometry_msgs::Pose>& posesV);
	Cluster(){};

	void calculateMaxDistPoint(void);
	void calculateCentroid(void);
	std::vector<geometry_msgs::Pose> getPoses(void) const { return poses; }
	void remove(const int i);
	int getSize(void) const { return poses.size(); }
	int setPoses(std::vector<geometry_msgs::Pose>& posesV) { poses.swap(posesV); }
	std::pair<int,double> getMaxDistPoint() const { return maxPoint; }

private:
	double distancePoints(const geometry_msgs::Pose& A, const geometry_msgs::Pose& B) const;

private:  
	std::vector<geometry_msgs::Pose> poses;	
	geometry_msgs::Pose centroid;
	std::pair<int, double> maxPoint;
};

#endif