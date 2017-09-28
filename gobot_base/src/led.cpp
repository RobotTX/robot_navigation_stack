#include <gobot_base/led.h>

//0x52 = Blue,0x47 = Red,0x42 = Green,0x4D = Magenta,0x43 = Cyan,0x59 = Yellow,0x57 = White, 0x00 = Off
#define RED 0x47
#define GREEN 0x42
#define BLUE 0x52
#define WHITE 0x57
#define YELLOW 0x59
#define OFF 0x00
#define CYAN 0x43
#define MAGENTA 0x4D

#define BUSY_STATE 99
#define LOCALIZE_STATE 11
#define BUMPER_STATE 10
#define GOAL_STATE 8
#define CHARGING_STATE 7
#define FREE_STATE 0

ros::Time last_time;

int current_state = FREE_STATE;

void setLedPermanent(std::vector<uint8_t> &color)
{
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
    cmd.request.data[9]=0x64;
    cmd.request.data[10]=0x1B;

    cmd.request.data[2]=color.size();
    for(int i=0;i<color.size();i++){
        cmd.request.data[3+i]=color[i];
    }
    ros::service::call("/gobot_base/setLed",cmd);
    last_time=ros::Time::now();
}

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    //LED permanent ON
    current_state=FREE_STATE;
    std::vector<uint8_t> color;
    switch(msg->status.status){
    case 2:
        //ROS_INFO("Goal PREEMPTED and disable teb_local_planner allow_init_with_backwards_motion.");
        color.push_back(BLUE);
        setLedPermanent(color);
        break;
    case 3:
        //ROS_INFO("Goal SUCCEED and disable teb_local_planner allow_init_with_backwards_motion.");
        color.push_back(GREEN);
        setLedPermanent(color);
        break;
    case 4:
        //ROS_INFO("Goal ABORTED and disable teb_local_planner allow_init_with_backwards_motion.");
        color.push_back(RED);
        setLedPermanent(color);
        break;
    default:
        //ROS_ERROR("Unknown goal status %d and disable teb_local_planner allow_init_with_backwards_motion.",msg->status.status);
        color.push_back(OFF);
        setLedPermanent(color);
        break;
    }
}

void goalGetCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
    if(current_state<GOAL_STATE){
        current_state=GOAL_STATE;
        //White Green LED Running
        std::vector<uint8_t> color;
        color.push_back(GREEN);
        color.push_back(WHITE);
        setLedRunning(color);
    }
}

void initialPoseCallback(const std_msgs::Int8::ConstPtr& msg){
    if(current_state<BUSY_STATE){
        std::vector<uint8_t> color;
        switch(msg->data){
        case -1:
            current_state=LOCALIZE_STATE;
            color.push_back(GREEN);
            color.push_back(WHITE);
            setLedRunning(color);
            break;
        case 0:
            current_state=FREE_STATE;
            color.push_back(RED);
            setLedPermanent(color);
            break;
        case 1:
            current_state=FREE_STATE;
            color.push_back(GREEN);
            setLedPermanent(color);
            break;
        case 2:
            current_state=FREE_STATE;
            color.push_back(BLUE);
            setLedPermanent(color);
            break;
        default:
            current_state=FREE_STATE;
            color.push_back(OFF);
            setLedPermanent(color);
            break;
        }
    }
}

void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if(msg->status_list.size()==1 && msg->status_list.back().status != 1){
        if(current_state==FREE_STATE){
            //Turn off LED if no goal for 3 mins
            if((ros::Time::now() - last_time).toSec()>300){
                std::vector<uint8_t> color;
                color.push_back(WHITE);
                setLedPermanent(color);
            }   
        } 
    }
}

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& msg){
    int n = msg->bumper1+msg->bumper2+msg->bumper3+msg->bumper4+msg->bumper5+msg->bumper6+msg->bumper7+msg->bumper8;
    if(n<8 && current_state<BUMPER_STATE && current_state!=CHARGING_STATE){
        current_state = BUMPER_STATE;
        //White Red LED Running
        std::vector<uint8_t> color;
        color.push_back(RED);
        color.push_back(WHITE);
        setLedRunning(color);
    }
    else if(n==8 && current_state==BUMPER_STATE){
        current_state = FREE_STATE;
    }
}

void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg){
    std::vector<uint8_t> color;
    if(msg->ChargingFlag && (current_state<CHARGING_STATE || current_state==BUMPER_STATE)){
        current_state = CHARGING_STATE;
        color.push_back(CYAN);
        color.push_back(WHITE);
        setLedRunning(color);
    }
    else if(!msg->ChargingFlag && current_state==CHARGING_STATE){
        color.push_back(CYAN);
        setLedPermanent(color);
        current_state = FREE_STATE;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "led");
    ros::NodeHandle nh;

    
    ros::Subscriber goalStatus = nh.subscribe("/move_base/status",1,goalStatusCallback);
    ros::Subscriber goalResult = nh.subscribe("/move_base/result",1,goalResultCallback);
    ros::Subscriber goalGet = nh.subscribe("/move_base/goal",1,goalGetCallback);
    ros::Subscriber bumpersSub = nh.subscribe("/bumpers_topic", 1, newBumpersInfo);
    ros::Subscriber initialPose = nh.subscribe("/gobot_recovery/find_initial_pose",1, initialPoseCallback);
    ros::Subscriber battery = nh.subscribe("/battery_topic",1, batteryCallback);

    ros::Duration(2.0).sleep();
    std::vector<uint8_t> color;
    color.push_back(RED);
    color.push_back(BLUE);
    color.push_back(GREEN);
    color.push_back(YELLOW);
    color.push_back(WHITE);
    setLedRunning(color);

    ros::spin();
    return 0;
}