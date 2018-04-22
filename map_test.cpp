#include <ros/ros.h>
#include <iostream>
#include <cstdint>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>
#include <thread>
#include <boost/asio.hpp>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>

#define HIGH_THRESHOLD 0.65*100
#define LOW_THRESHOLD 0.196*100

#define WHITE 255
#define GREY 205
#define BLACK 0
#define COLO 255
using namespace std;

std::string mapFile;
double map_resolution = 0.02;
double map_origin_x = -1;
double map_origin_y = -1;

uint8_t **ori_map, **processed_map;

std::vector<uint8_t> compress(const std::vector<int8_t> map, const int map_width, const int map_height, const double map_resolution, const double _map_orixin_x, const double _map_orixin_y){
	std::vector<uint8_t> my_map;

    for(int i = 0; i < std::to_string(map_width).size(); ++i)
        my_map.push_back(static_cast<uint8_t>(std::to_string(map_width).at(i)));
    my_map.push_back(static_cast<uint8_t>(' '));

    for(int i = 0; i < std::to_string(map_height).size(); ++i)
        my_map.push_back(static_cast<uint8_t>(std::to_string(map_height).at(i)));
    my_map.push_back(static_cast<uint8_t>(' '));
    
    for(int i = 0; i < std::to_string(map_resolution).size(); ++i)
        my_map.push_back(static_cast<uint8_t>(std::to_string(map_resolution).at(i)));
    my_map.push_back(static_cast<uint8_t>(' '));
    
    for(int i = 0; i < std::to_string(_map_orixin_x).size(); ++i)
        my_map.push_back(static_cast<uint8_t>(std::to_string(_map_orixin_x).at(i)));
    my_map.push_back(static_cast<uint8_t>(' '));
    
    for(int i = 0; i < std::to_string(_map_orixin_y).size(); ++i)
        my_map.push_back(static_cast<uint8_t>(std::to_string(_map_orixin_y).at(i)));
    
    my_map.push_back(252);
    my_map.push_back(252);
    my_map.push_back(252);
    my_map.push_back(252);
    my_map.push_back(252);


	int map_size = map_width * map_height;
	int last = GREY;
	uint32_t count=0;
	int curr = 0;
    ROS_INFO("(COMPRESS) Map size %d vs %lu", map_size, map.size());
	for(size_t i = 0; i < map_size; i++){
        if(i >= map.size())
            curr = GREY;
        else
            curr = map.at(i);

        if(curr < 0)
            curr = GREY;
        else if(curr < LOW_THRESHOLD)
            curr = WHITE;
        else if(curr < HIGH_THRESHOLD)
            curr = GREY;
        else 
            curr = BLACK;

		if(curr != last){
			my_map.push_back(static_cast<uint8_t>(last));
			my_map.push_back((count & 0xff000000) >> 24);
			my_map.push_back((count & 0x00ff0000) >> 16);
			my_map.push_back((count & 0x0000ff00) >> 8);
			my_map.push_back((count & 0x000000ff));

			last = curr;
			count = 0;
		}
		count++;
	}

	my_map.push_back(static_cast<uint8_t>(last));
	my_map.push_back((count & 0xff000000) >> 24);
	my_map.push_back((count & 0x00ff0000) >> 16);
	my_map.push_back((count & 0x0000ff00) >> 8);
	my_map.push_back((count & 0x000000ff));

	// the user knows that when 254 is encountered a map has entirely been received
	my_map.push_back(254);
	my_map.push_back(254);
	my_map.push_back(254);
	my_map.push_back(254);
    my_map.push_back(253);

    ROS_INFO("(COMPRESS) Map after compression size %zu", my_map.size());

	return my_map;
}

void sharpEdge(const std::vector<int8_t> map, const int map_width, const int map_height){
    ori_map = new uint8_t *[map_height];
	processed_map = new uint8_t *[map_height];
	
	for(int i=0;i<map_height;i++){
		ori_map[i] = new uint8_t [map_width];
		processed_map[i] = new uint8_t [map_width];
	}

    int index = 0, curr = 0;
    for(int i=0;i<map_height;i++){
        for(int j=0;j<map_width;j++){
            curr = map.at(index);
            if(curr < 0)
                curr = GREY;
            else if(curr < LOW_THRESHOLD)
                curr = WHITE;
            else if(curr < HIGH_THRESHOLD)
                curr = GREY;
            else 
                curr = BLACK;
            index++;

            ori_map[i][j] = static_cast<uint8_t>(curr);
			processed_map[i][j] = static_cast<uint8_t>(curr);
        }
    }

int dis = 5;
	int sh,eh,sw,ew;
	//断点连接
	sh = dis, eh = map_height - dis, sw = dis, ew = map_width - dis;
	for(int i=sh;i<eh;i++){
		for(int j=sw;j<ew;j++){
			if(processed_map[i][j]!=BLACK){
				int lb = 0, rb = 0;
				for(int k=j-dis;k<j;k++){
					if(processed_map[i][k]==BLACK){
						lb++;
						break;
					}
				}
				for(int k=j+1;k<=j+dis;k++){
					if(processed_map[i][k]==BLACK){
						rb++;
						break;
					}
				}
				if(lb>0 && rb>0){
					processed_map[i][j]=BLACK;
					continue;
				}

				lb = 0, rb = 0;
				for(int k=i-dis;k<i;k++){
					if(processed_map[k][j]==BLACK){
						lb++;
						break;
					}
				}
				for(int k=i+1;k<=i+dis;k++){
					if(processed_map[k][j]==BLACK){
						rb++;
						break;
					}
				}
				if(lb>0 && rb>0){
					processed_map[i][j]=BLACK;
					continue;
				}
			}
		}
	}


	//去毛边
	for(int i=1;i<map_height-1;i++){
        for(int j=1;j<map_width-1;j++){
			if(processed_map[i][j]==BLACK){
				int count = 0;
				for(int k=-1;k<=1;k++){
					for(int l=-1;l<=1;l++){
						count += processed_map[i+k][j+l]==BLACK?1:0;
					}
				}

				if(count==4){
					int neibours = 0;
					for(int m=-1;m<=1;m++){
						neibours += processed_map[i-1][j+m]==BLACK?1:0;
					}
					if(neibours==3){
						processed_map[i][j]=COLO;
						continue;
					}

					neibours = 0;
					for(int m=-1;m<=1;m++){
						neibours += processed_map[i+1][j+m]==BLACK?1:0;
					}
					if(neibours==3){
						processed_map[i][j]=COLO;
						continue;
					}

					neibours = 0;
					for(int m=-1;m<=1;m++){
						neibours += processed_map[i+m][j-1]==BLACK?1:0;
					}
					if(neibours==3){
						processed_map[i][j]=COLO;
						continue;
					}

					neibours = 0;
					for(int m=-1;m<=1;m++){
						neibours += processed_map[i+m][j+1]==BLACK?1:0;
					}
					if(neibours==3){
						processed_map[i][j]=COLO;
						continue;
					}
				}
			}
		}
	}

	/*
	sh = 1, eh = map_height-1, sw = 1, ew = map_width - 1;
	//补全黑色
	//left->right
	for(int i=sh;i<eh;i++){
		for(int j=sw;j<ew;j++){
			if(processed_map[i][j]!=BLACK){
				int count = 0;
				for(int k=-1;k<=1;k++){
					for(int l=-1;l<=1;l++){
						count += processed_map[i+k][j+l]==BLACK?1:0;
					}
				}
				if(count>=4){
					processed_map[i][j]=BLACK;
				}
			}
		}
	}
	
	/*
	//right->left
	sh = 1, eh = map_height-1, sw = map_width-2, ew = 1;
	for(int i=sh;i<eh;i++){
		for(int j=sw;j<ew;j--){
			if(processed_map[i][j]!=BLACK){
				int count = 0;
				for(int k=-1;k<=1;k++){
					for(int l=-1;l<=1;l++){
						count += processed_map[i+k][j+l]==BLACK?1:0;
					}
				}
				if(count>=4){
					processed_map[i][j]=BLACK;
				}
			}
		}
	}

	//bottom->top
	sh = 1, eh = map_height-1, sw = 1, ew = map_width-1;
	for(int j=sw;j<ew;j++){
		for(int i=sh;i<eh;i++){
			if(processed_map[i][j]!=BLACK){
				int count = 0;
				for(int k=-1;k<=1;k++){
					for(int l=-1;l<=1;l++){
						count += processed_map[i+k][j+l]==BLACK?1:0;
					}
				}
				if(count>=4){
					processed_map[i][j]=BLACK;
				}
			}
		}
	}

	//top->bottom
	sh = map_height-2, eh = 1, sw = 1, ew = map_width-1;
	for(int j=sw;j<ew;j++){
		for(int i=sh;i>=eh;i--){
			if(processed_map[i][j]!=BLACK){
				int count = 0;
				for(int k=-1;k<=1;k++){
					for(int l=-1;l<=1;l++){
						count += processed_map[i+k][j+l]==BLACK?1:0;
					}
				}
				if(count>=4){
					processed_map[i][j]=BLACK;
				}
			}
		}
	}
	*/
	
    std::ofstream ofs;
    ofs.open(mapFile, std::ofstream::out | std::ofstream::trunc);
    if(ofs.is_open()){
        /// pgm file header
        ofs << "P5" << std::endl << map_width << " " << map_height << std::endl << "255" << std::endl;
        /// writes every single pixel to the pgm file
        for(int i=map_height-1;i>=0;i--){
            for(int j=0;j<map_width;j++){
                ofs << processed_map[i][j];
            }
        }
        ofs << std::endl;
        ofs.close();
    }
	ROS_INFO("Saved map");
	delete [] ori_map;
	delete [] processed_map;
}

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    ROS_INFO("Map: width: %d, height: %d,  reso: %f, ori_x: %f, ori_y: %f", 
    msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
    ROS_INFO("Data size: %zu, w*h: %d", msg->data.size(), msg->info.width * msg->info.height);
    compress(msg->data, msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);
    sharpEdge(msg->data, msg->info.width, msg->info.height);
}

void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg){
	map_resolution = msg->resolution;
    map_origin_x = msg->origin.position.x;
    map_origin_y = msg->origin.position.y;
    ROS_INFO("Receive map metadata: reso: %f, ori_x: %f, ori_y: %f",map_resolution,map_origin_x,map_origin_y);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "map_test");
    ros::NodeHandle n;

	ROS_INFO("Complete wait");
    n.getParam("output1", mapFile);
    ros::Subscriber sub_meta = n.subscribe("/map_metadata", 1, getMetaData);
    ros::Subscriber sub_map = n.subscribe("/map", 1, getMap);

    ros::spin();

    return 0;
}