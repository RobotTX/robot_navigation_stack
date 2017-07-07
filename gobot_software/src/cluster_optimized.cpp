#include "cluster_optimized.h"

Cluster::Cluster(const std::vector<geometry_msgs::Pose>& posesV): poses(posesV)
{}

double Cluster::distancePoints(const geometry_msgs::Pose& A, const geometry_msgs::Pose& B) const {
	double xdist = pow((B.position.x - A.position.x), 2);
	double ydist = pow((B.position.y - A.position.y), 2);
	double zdist = pow((B.position.z - A.position.z), 2);
	return xdist + ydist + zdist;
}

void Cluster::calculateMaxDistPoint(void){
	
	double maxDist(-100000.0);
	double dist(0.0);
	int maxID(-1);

	for (size_t i = 0; i < poses.size(); i++){	
		dist = distancePoints(poses[i], centroid);
		if (dist > maxDist){
			maxDist = dist;
			maxID = i;
		}
	}

	maxPoint.first = maxID;
	maxPoint.second = maxDist;
}	

void Cluster::calculateCentroid(void) {
	
	double sumx(0.0);
	double sumy(0.0);
	double sumz(0.0);

	double maxDist(-1000000.0);
	double dist(0.0);

	for(size_t i = 0; i < poses.size(); i++){

		sumx = sumx + poses.at(i).position.x;
		sumy = sumy + poses.at(i).position.y;
		sumz = sumz + poses.at(i).position.z;
	}

	centroid.position.x = sumx / (poses.size()-1);
	centroid.position.y = sumy / (poses.size()-1);
	centroid.position.z = sumz / (poses.size()-1);
}

void Cluster::remove(const int i){
	if(i < poses.size())
		poses.erase(poses.begin() + i);
}