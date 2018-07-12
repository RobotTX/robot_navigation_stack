#include <gobot_sensors2pc/sonars2pc.hpp>

ros::Publisher sonar_range1, sonar_range2, sonar_range3, sonar_range4;
std::string rear_right_frame, rear_left_frame, front_right_frame, front_left_frame;
double SONAR_MIN=0, SONAR_MAX=1.5, SONAR_VIEW_ANGLE=0.2;
bool use_pc = true;

double sonarRawToRange(double sonar_raw){
    if(sonar_raw>SONAR_MAX)
        return SONAR_MAX;
    else
        return sonar_raw;
}

void sonarCallback(const gobot_msg_srv::SonarMsg::ConstPtr& sonars){
    if(use_pc){
        //transform sonar raw data into sensor_msgs::range format
        sensor_msgs::Range sonar_msg;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        sonar_msg.field_of_view = SONAR_VIEW_ANGLE;
        sonar_msg.min_range = SONAR_MIN;
        sonar_msg.max_range = SONAR_MAX;

        header.frame_id = front_right_frame;
        sonar_msg.header = header;
        sonar_msg.range = sonarRawToRange(sonars->distance1/100.0);
        sonar_range1.publish(sonar_msg);

        header.frame_id = front_left_frame;
        sonar_msg.header = header;
        sonar_msg.range = sonarRawToRange(sonars->distance2/100.0);
        sonar_range2.publish(sonar_msg);

        header.frame_id = rear_left_frame;
        sonar_msg.header = header;
        sonar_msg.range = sonarRawToRange(sonars->distance3/100.0);
        sonar_range3.publish(sonar_msg);

        header.frame_id = rear_right_frame;
        sonar_msg.header = header;
        sonar_msg.range = sonarRawToRange(sonars->distance4/100.0);
        sonar_range4.publish(sonar_msg);
    }
}

void initParams(){
    ros::NodeHandle nh;
    nh.param("rear_right_frame", rear_right_frame, std::string("rear_right_sonar"));
    nh.param("rear_left_frame", rear_left_frame, std::string("rear_left_sonar"));
    nh.param("front_right_frame", front_right_frame, std::string("front_right_sonar"));
    nh.param("front_left_frame", front_left_frame, std::string("front_left_sonar"));

    nh.getParam("SONAR_MIN", SONAR_MIN);
    nh.getParam("SONAR_MAX", SONAR_MAX);
    nh.getParam("SONAR_VIEW_ANGLE", SONAR_VIEW_ANGLE);
    nh.getParam("USE_SONAR_PC", use_pc);

    std::cout << "(SONAR2PC::initParams) SONAR MIN:"<<SONAR_MIN<<" SONAR MAX:"<<SONAR_MAX<<" SONAR SONAR_VIEW_ANGLE:"<<SONAR_VIEW_ANGLE<<std::endl;
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "sonars2pc");
    ros::NodeHandle nh;

    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ros::service::waitForService("/gobot_startup/pose_ready", ros::Duration(90.0));
    //Startup end
    
    initParams();

    sonar_range1 = nh.advertise<sensor_msgs::Range>("/gobot_pc/sonar_range1", 1);
    sonar_range2 = nh.advertise<sensor_msgs::Range>("/gobot_pc/sonar_range2", 1);
    sonar_range3 = nh.advertise<sensor_msgs::Range>("/gobot_pc/sonar_range3", 1);
    sonar_range4 = nh.advertise<sensor_msgs::Range>("/gobot_pc/sonar_range4", 1);

    // get the sonars information
    ros::Subscriber sonarSub = nh.subscribe("/gobot_base/sonar_topic", 1, sonarCallback);

    ros::spin();

    return 0;
}