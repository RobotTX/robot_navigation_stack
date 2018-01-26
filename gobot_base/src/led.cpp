#include <gobot_base/led.h>

//0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
#define RED 0x47
#define GREEN 0x42
#define BLUE 0x52
#define WHITE 0x57
#define YELLOW 0x59
#define CYAN 0x43
#define MAGENTA 0x4D
#define OFF 0x00

#define LOW_BATTERY_STAGE 99
#define LOST_STAGE 12
#define COMPLETE_STAGE 11
#define BUMPER_STAGE 10
#define GOAL_STAGE 9
#define CHARGING_STAGE 8
#define LOCALIZE_STAGE 7
#define FREE_STAGE 0

/*
-----LED Running-----
green white - go to path point/auto scan
red white - bumper collision
cyan white - charging battery 75%~100%
yellow white - charging battery 25%~75%
magenta white - charging battery 0%~25%
cyan yellow - go to docking
red blue - get lost

-----LED Permanent-----
green - reach path point/found initial pose/complete auto scan
red - abort path point/fail to docking/not found initial pose
blue - cancel path point/cancel docking
cyan - battery 75%~100%
yellow - battery 25%~75%
magenta - battery 0%~25%
white - scan mode/find pose
*/

ros::Time last_time;

int current_stage = FREE_STAGE;
int charging_state = -1, pre_charing_state=-1;
bool charging = false;

gobot_msg_srv::GetGobotStatus get_gobot_status;
ros::ServiceClient getGobotStatusSrv;
std_srvs::Empty empty_srv;

void setSound(int num,int time_on, int time_off){
    gobot_msg_srv::SetInt sound_num;
    sound_num.request.data.push_back(num);
    sound_num.request.data.push_back(time_on);
    if(time_off!=0)
        sound_num.request.data.push_back(time_off);

    ros::service::call("/gobot_base/setSound",sound_num);
}

void setLedPermanent(std::vector<uint8_t> &color){
    gobot_msg_srv::LedStrip cmd;
    cmd.request.data[0]=0xB0;
    cmd.request.data[1]=0x03;
    cmd.request.data[2]=0x00;
    cmd.request.data[3]=0x00;
    cmd.request.data[4]=0x00;
    cmd.request.data[5]=0x00;
    cmd.request.data[6]=0x00;
    cmd.request.data[7]=0x00;
    cmd.request.data[8]=0x03;
    cmd.request.data[9]=0xE8;
    cmd.request.data[10]=0x1B;

    cmd.request.data[2]=color.size();
    for(int i=0;i<color.size();i++){
        cmd.request.data[3+i]=color[i];
    }
    ros::service::call("/gobot_base/setLed",cmd);
    last_time=ros::Time::now();
}

void setLedRunning(std::vector<uint8_t> &color)
{
    gobot_msg_srv::LedStrip cmd;
    cmd.request.data[0]=0xB0;
    cmd.request.data[1]=0x01;
    cmd.request.data[2]=0x00;
    cmd.request.data[3]=0x00;
    cmd.request.data[4]=0x00;
    cmd.request.data[5]=0x00;
    cmd.request.data[6]=0x00;
    cmd.request.data[7]=0x00;
    cmd.request.data[8]=0x00;
    cmd.request.data[9]=0x96;   //150 = 0x64
    cmd.request.data[10]=0x1B;

    cmd.request.data[2]=color.size();
    for(int i=0;i<color.size();i++){
        cmd.request.data[3+i]=color[i];
    }
    ros::service::call("/gobot_base/setLed",cmd);
    last_time=ros::Time::now();
}


void setLedSlowRunning(std::vector<uint8_t> &color)
{
    gobot_msg_srv::LedStrip cmd;
    cmd.request.data[0]=0xB0;
    cmd.request.data[1]=0x01;
    cmd.request.data[2]=0x00;
    cmd.request.data[3]=0x00;
    cmd.request.data[4]=0x00;
    cmd.request.data[5]=0x00;
    cmd.request.data[6]=0x00;
    cmd.request.data[7]=0x00;
    cmd.request.data[8]=0x01;
    cmd.request.data[9]=0xF4;
    cmd.request.data[10]=0x1B;

    cmd.request.data[2]=color.size();
    for(int i=0;i<color.size();i++){
        cmd.request.data[3+i]=color[i];
    }
    ros::service::call("/gobot_base/setLed",cmd);
    last_time=ros::Time::now();
}


void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& msg){
    int n = msg->bumper1+msg->bumper2+msg->bumper3+msg->bumper4+msg->bumper5+msg->bumper6+msg->bumper7+msg->bumper8;

    if(n<8 && current_stage<BUMPER_STAGE){
        getGobotStatusSrv.call(get_gobot_status);
        if(get_gobot_status.response.status!=15){
            current_stage = BUMPER_STAGE;
            //White Red LED Running
            std::vector<uint8_t> color;
            color.push_back(RED);
            color.push_back(WHITE);
            setLedRunning(color);
            setSound(3,1);
        }
    }
    else if(n==8 && current_stage==BUMPER_STAGE){
        current_stage = FREE_STAGE;
        setSound(1,1);
    }
}

void lostCallback(const std_msgs::Int8::ConstPtr& msg){
    std::vector<uint8_t> color;
    if(msg->data==1 && current_stage<LOST_STAGE){
        current_stage=LOST_STAGE;
        color.push_back(RED);
        color.push_back(BLUE);
        setLedRunning(color);
    }
    else if(msg->data==0 && current_stage==LOST_STAGE){
        current_stage=FREE_STAGE;
        color.push_back(WHITE);
        setLedPermanent(color);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "led");
    ros::NodeHandle nh;

    ros::service::waitForService("/gobot_base/setLed", ros::Duration(30));

    ros::Subscriber bumpersSub = nh.subscribe("/gobot_base/bumpers_topic", 1, newBumpersInfo);
    ros::Subscriber lostRobot = nh.subscribe("/gobot_recovery/lost_robot",1,lostCallback);

    getGobotStatusSrv = nh.serviceClient<gobot_msg_srv::GetGobotStatus>("/gobot_status/get_gobot_status");
    last_time=ros::Time::now();

    ros::spin();
    return 0;
}
