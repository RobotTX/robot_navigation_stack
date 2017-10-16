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

ros::Time last_time;

int current_stage = FREE_STAGE;
int charging_state = 0, pre_charing_state=-1;

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
    if(current_stage<COMPLETE_STAGE){
        std::vector<uint8_t> color;
        switch(msg->status.status){
            case 2:
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
                color.push_back(MAGENTA);
                setLedPermanent(color);
                break;
        }
        current_stage=FREE_STAGE;
    }
}

void goalGetCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
    if(current_stage<GOAL_STAGE){
        current_stage=GOAL_STAGE;
        //White Green LED Running
        std::vector<uint8_t> color;
        color.push_back(GREEN);
        color.push_back(WHITE);
        setLedRunning(color);
    }
}

void goalCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg){
    if(current_stage==GOAL_STAGE && current_stage<COMPLETE_STAGE){
        std::vector<uint8_t> color;
        //ROS_INFO("Goal Cancelled and disable teb_local_planner allow_init_with_backwards_motion.");
        color.push_back(BLUE);
        setLedPermanent(color);
        current_stage=FREE_STAGE;
    }
}

void initialPoseResultCallback(const std_msgs::Int8::ConstPtr& msg){
    std::vector<uint8_t> color;
    if(current_stage<LOCALIZE_STAGE){
        switch(msg->data){
            //START
            case -1:
                current_stage=FREE_STAGE;
                color.push_back(BLUE);
                color.push_back(WHITE);
                setLedRunning(color);
                break;
            case 0:
            //NOT FOUND
                current_stage=FREE_STAGE;
                color.push_back(RED);
                setLedPermanent(color);
                break;
            case 1:
            //FOUND
                current_stage=FREE_STAGE;
                color.push_back(GREEN);
                setLedPermanent(color);
                break;
            case 2:
            //CANCEL
                current_stage=FREE_STAGE;
                color.push_back(BLUE);
                setLedPermanent(color);
                break;
            default:
                current_stage=FREE_STAGE;
                color.push_back(OFF);
                setLedPermanent(color);
                break;
        }
    }
}

void newBumpersInfo(const gobot_msg_srv::BumperMsg::ConstPtr& msg){
    int n = msg->bumper1+msg->bumper2+msg->bumper3+msg->bumper4+msg->bumper5+msg->bumper6+msg->bumper7+msg->bumper8;
    //if(n<8 && current_stage<BUMPER_STAGE && current_stage!=CHARGING_STAGE){
    if(n<8 && current_stage<BUMPER_STAGE){
        current_stage = BUMPER_STAGE;
        //White Red LED Running
        std::vector<uint8_t> color;
        color.push_back(RED);
        color.push_back(WHITE);
        setLedRunning(color);
    }
    else if(n==8 && current_stage==BUMPER_STAGE){
        current_stage = FREE_STAGE;
    }
}

void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& msg){
    if(msg->Percentage<=0.25)
        charging_state=0;
    else if(msg->Percentage<=0.75)
        charging_state=1;
    else if(msg->Percentage<1.0)
        charging_state=2;
    else if(msg->Percentage==1.0)
        charging_state=3;

    std::vector<uint8_t> color;
    if(msg->ChargingFlag && (current_stage==LOW_BATTERY_STAGE || current_stage<CHARGING_STAGE || charging_state!=pre_charing_state)){
        current_stage = CHARGING_STAGE;
        pre_charing_state=charging_state;
        switch (charging_state){
            case 0:
            color.push_back(MAGENTA);
            color.push_back(WHITE);
            setLedRunning(color);
            break;

            case 1:
            color.push_back(YELLOW);
            color.push_back(WHITE);
            setLedRunning(color);
            break;

            case 2:
            color.push_back(CYAN);
            color.push_back(WHITE);
            setLedRunning(color);
            break;

            case 3:
            color.push_back(CYAN);
            setLedPermanent(color);
            break;
        }
    }
    else if(!msg->ChargingFlag && current_stage==CHARGING_STAGE){
        current_stage = FREE_STAGE;
        pre_charing_state = -1;
        switch (charging_state){
            case 0:
            color.push_back(MAGENTA);
            setLedPermanent(color);
            break;

            case 1:
            color.push_back(YELLOW);
            setLedPermanent(color);
            break;

            case 2:
            color.push_back(CYAN);
            setLedPermanent(color);
            break;

            case 3:
            color.push_back(CYAN);
            setLedPermanent(color);
            break;
        }
    }
}

void explorationCallback(const std_msgs::Int8::ConstPtr& msg){
    std::vector<uint8_t> color;
    if(current_stage<COMPLETE_STAGE){
        switch(msg->data){
            //hector exploration completed
            case 1:
                current_stage=FREE_STAGE;
                color.push_back(RED);
                color.push_back(BLUE);
                color.push_back(GREEN);
                setLedRunning(color);
                break;
            default:
                current_stage=FREE_STAGE;
                break;
        }
    }
}

void lostCallback(const std_msgs::Int8::ConstPtr& msg){
    std::vector<uint8_t> color;
    if(msg->data==1 && current_stage<LOST_STAGE){
        current_stage=LOST_STAGE;
        color.push_back(RED);
        color.push_back(BLUE);
        color.push_back(GREEN);
        setLedRunning(color);
    }
    else if(msg->data==0 && current_stage==LOST_STAGE){
        current_stage=FREE_STAGE;
        color.push_back(BLUE);
        setLedPermanent(color);
    }
}

void timerCallback(const ros::TimerEvent&){
    std::vector<uint8_t> color;
    if (charging_state == 0 && current_stage != CHARGING_STAGE && current_stage<LOW_BATTERY_STAGE){
        current_stage = LOW_BATTERY_STAGE;
        color.push_back(MAGENTA);
        setLedPermanent(color);
    }
    //Show battery status if no goal for 5 mins
    if((ros::Time::now() - last_time).toSec()>300.0 && current_stage==FREE_STAGE){
        switch (charging_state){
            case 1:
            color.push_back(YELLOW);
            setLedPermanent(color);
            break;

            case 2:
            color.push_back(CYAN);
            setLedPermanent(color);
            break;

            case 3:
            color.push_back(CYAN);
            setLedPermanent(color);
            break;
        }
    }   
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "led");
    ros::NodeHandle nh;

    ros::Timer timer = nh.createTimer(ros::Duration(5), timerCallback);

    ros::Subscriber goalResult = nh.subscribe("/move_base/result",1,goalResultCallback);
    ros::Subscriber goalGet = nh.subscribe("/move_base/goal",1,goalGetCallback);
    ros::Subscriber goalCancel = nh.subscribe("/move_base/cancel",1,goalCancelCallback);
    ros::Subscriber initialPoseResult = nh.subscribe("/gobot_recovery/find_initial_pose",1, initialPoseResultCallback);
    ros::Subscriber bumpersSub = nh.subscribe("/gobot_base/bumpers_topic", 1, newBumpersInfo);
    ros::Subscriber battery = nh.subscribe("/gobot_base/battery_topic",1, batteryCallback);
    ros::Subscriber exploration = nh.subscribe("/move_base_controller/exploration_result",1,explorationCallback);
    ros::Subscriber lostRobot = nh.subscribe("/gobot_recovery/lost_robot",1,lostCallback);
    last_time=ros::Time::now();

    ros::spin();
    return 0;
}