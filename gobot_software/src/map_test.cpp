#include "gobot_software/read_new_map.hpp"

#define HIGH_THRESHOLD 0.65*100
#define LOW_THRESHOLD 0.196*100

#define WHITE 255
#define GREY 205
#define BLACK 0

std::string mapFile;
double map_resolution = 0.02;
double map_origin_x = -1;
double map_origin_y = -1;

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
    uint8_t ori_map[map_height][map_width];
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
        }
    }
    uint8_t vertical_map[map_height][map_width];
    uint8_t processed_map[map_height][map_width];

    //process vertically
    for(int i=0;i<map_height;i++){
        for(int j=0;j<map_width;j++){
            int sum = 0;
            if(ori_map[i][j]==GREY || ori_map[i][j]==BLACK){
                processed_map[i][j] = ori_map[i][j];
            }
            else{
                if(j<4){
                    for(int k=0;k<9;k++){
                        sum +=  ori_map[i][k];
                    }
                }
                else if(j>map_width-5){
                    for(int k=map_width-1;k>map_width-10;k--){
                        sum +=  ori_map[i][k];
                    }
                }
                else{
                    for(int k=j-4; k<=j+4;k++){
                        sum +=  ori_map[i][k];
                    }
                }
                sum = sum/9;
                if(sum>=GREY)
                    processed_map[i][j] = static_cast<uint8_t>(WHITE);
                else
                    processed_map[i][j] = static_cast<uint8_t>(BLACK);
            }

            vertical_map[i][j] = processed_map[i][j];
        }
    }

    //process horizontally
    for(int i=0;i<map_width;i++){
        for(int j=0;j<map_height;j++){
            int sum = 0;
            if(vertical_map[j][i]==GREY || vertical_map[j][i]==BLACK){
                processed_map[j][i] = vertical_map[j][i];
            }
            else{
                if(j<4){
                    for(int k=0;k<9;k++){
                        sum +=  vertical_map[k][i];
                    }
                }
                else if(j>map_height-5){
                    for(int k=map_height-1;k>map_height-10;k--){
                        sum +=  vertical_map[k][i];
                    }
                }
                else{
                    for(int k=j-4; k<=j+4;k++){
                        sum +=  vertical_map[k][i];
                    }
                }
                sum = sum/9;
                if(sum>=GREY)
                    processed_map[j][i] = static_cast<uint8_t>(WHITE);
                else
                    processed_map[j][i] = static_cast<uint8_t>(BLACK);
            }
        }
    }

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
        ROS_INFO("(MAP_READ) New map pgm file created in %s", mapFile.c_str());
    }
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
    n.getParam("map_image_used", mapFile);
    ros::Subscriber sub_meta = n.subscribe("/map_metadata", 1, getMetaData);
    ros::Subscriber sub_map = n.subscribe("/map", 1, getMap);

    ros::spin();

    return 0;
}
