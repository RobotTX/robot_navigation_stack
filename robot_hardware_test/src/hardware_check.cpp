#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>	
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <gobot_msg_srv/robot_msgs.h>
#include <gobot_msg_srv/set_robot_class.h>
#include <gobot_msg_srv/get_robot_class.h>

using namespace std;
//timeout for each test
const int TIME_OUT = 30;
const double DELAY = 3.0;
const int MOTOR_SPD = 12;

vector<string> test_log;

robot_class::SetRobot SetRobot;

int bumpers_data[8] = {-1};
int sonar_data[4] = {-1};
int ir_data[3] = {-1};
int cliff_data[4] = {-1};
int proxi_data[2] = {-1};
double load_data = -1;
bool battery_charging = false;
int left_vel = 0, right_vel = 0;
bool enable_motor = false;

void velCallback(const geometry_msgs::Twist::ConstPtr& twist){
    if(enable_motor){
        if(twist->linear.x > 0){
            left_vel = MOTOR_SPD;
            right_vel = MOTOR_SPD;
        }
        else if(twist->linear.x < 0){
            left_vel = -MOTOR_SPD;
            right_vel = -MOTOR_SPD;
        }
        //turn left
        else if(twist->angular.z > 0){
            left_vel = -MOTOR_SPD;
            right_vel = MOTOR_SPD;
        }
        else if(twist->angular.z < 0){
            left_vel =  MOTOR_SPD;
            right_vel = -MOTOR_SPD;
        }
        else{
            left_vel =  0;
            right_vel = 0;
        }
    }
}

void bumperCallback(const gobot_msg_srv::BumperMsg::ConstPtr& bumpers){
    bumpers_data[0] = bumpers->bumper1;
    bumpers_data[1] = bumpers->bumper2;
    bumpers_data[2] = bumpers->bumper3;
    bumpers_data[3] = bumpers->bumper4;
    bumpers_data[4] = bumpers->bumper5;
    bumpers_data[5] = bumpers->bumper6;
    bumpers_data[6] = bumpers->bumper7;
    bumpers_data[7] = bumpers->bumper8;
}

void sonarCallback(const gobot_msg_srv::SonarMsg::ConstPtr& sonars){
    sonar_data[0] = sonars->distance1;
    sonar_data[1] = sonars->distance2;
    sonar_data[2] = sonars->distance3;
    sonar_data[3] = sonars->distance4;
}


void irCallback(const gobot_msg_srv::IrMsg::ConstPtr& irs){
    ir_data[0] = irs->leftSignal;
    ir_data[1] = irs->rearSignal;
    ir_data[2] = irs->rightSignal;
}

void cliffCallback(const gobot_msg_srv::CliffMsg::ConstPtr& cliffs){
    cliff_data[0] = cliffs->cliff1/10;
    cliff_data[1] = cliffs->cliff2/10;
    cliff_data[2] = cliffs->cliff3/10;
    cliff_data[3] = cliffs->cliff4/10;
}

void proxiCallback(const gobot_msg_srv::ProximityMsg::ConstPtr& proximity){
    proxi_data[0] = proximity->signal1;
    proxi_data[1] = proximity->signal2;
}


void loadCallback(const gobot_msg_srv::WeightMsg::ConstPtr& weight){
    load_data = weight->weight;
}

void batteryCallback(const gobot_msg_srv::BatteryMsg::ConstPtr& battery){
    battery_charging = battery->ChargingFlag;
}

bool bumpersTest(int index){
    int timer = 0, range = 8;
    bool triger_sensor = false;
    string sensor_name = "bumper";
    if(index!=0){
        range = index;
    }
    else{
        index = 1;
    }
    for(int i=index-1; i<range; i++){
        timer = 0;
        triger_sensor = false;
        while(ros::ok() && timer<TIME_OUT){
            ros::spinOnce(); 
            cout<<endl;
            cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<"  ##"<< i+1 <<"##  within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
            cout<<"Current "<<sensor_name<<" reading: Front ["<<bumpers_data[0]<<", "<<bumpers_data[1]<<", "<<bumpers_data[2]<<", "<<bumpers_data[3]<<
            "].   Back ["<<bumpers_data[4]<<", "<<bumpers_data[5]<<", "<<bumpers_data[6]<<", "<<bumpers_data[7]<<"]"<<endl;  
            
            //bumper value 0 means collision
            if(bumpers_data[i]==0){
                cout<<endl;
                SetRobot.setSound(1,1);
                triger_sensor = true;
                cout<<"##################################################"<<endl;
                cout<<"Detected "<<sensor_name<<"  ##"<< i+1 << "##  with value  ##"<<bumpers_data[i]<<"##"<<endl;
                cout<<"##################################################"<<endl<<endl;
                ros::Duration(DELAY).sleep(); 
                break;
            }
            ros::Duration(1.0).sleep(); 
            timer++;
        }

        if(!triger_sensor){
            cout<<"##################################################"<<endl;
            cout<<"Unable to detect "<<sensor_name<<"  ##"<< i+1 << "##.  Quit "<<sensor_name<<" test" <<endl;
            cout<<"##################################################"<<endl<<endl;
            return false;
        }
    }
    
    return true;
}

bool sonarsTest(int index){
    int timer = 0, range = 4, test_dis = 50;
    bool triger_sensor = false;
    string sensor_name = "sonar";
    if(index!=0){
        range = index;
    }
    else{
        index = 1;
    }

    for(int i=index-1; i<range; i++){
        test_dis = 50;
        timer = 0;
        triger_sensor = false;
        while(ros::ok() && timer<TIME_OUT){
            ros::spinOnce(); 
            cout<<endl;
            if(test_dis==50)
                cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<"  ##"<< i+1 <<"##  less than "<< test_dis<< "cm within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
            else
                cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<"  ##"<< i+1 <<"##  larger than "<< test_dis<< "cm within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
            cout<<"Current "<<sensor_name<<" reading: 1: "<<sonar_data[0]<<", 2: "<<sonar_data[1]<<", 3: "<<sonar_data[2]<<", 4: "<<sonar_data[3]<<endl;  
            
            //sonar value 0 means out of range
            if(sonar_data[i] > 0){
                if((sonar_data[i]<test_dis && test_dis==50) || (sonar_data[i]>test_dis && test_dis==100)){
                    cout<<endl;
                    SetRobot.setSound(1,1);
                    triger_sensor = true;
                    cout<<"##################################################"<<endl;
                    cout<<"Detected "<<sensor_name<<"  ##"<< i+1 << "##  wtih value  ##"<< sonar_data[i]<<"##"<<endl; 
                    cout<<"##################################################"<<endl<<endl; 
                    ros::Duration(DELAY).sleep(); 
                    if(test_dis==50){
                        timer=0;
                        triger_sensor = false;
                        test_dis = 100;
                    }
                    else{
                        break;
                    }
                }
            }
                ros::Duration(1.0).sleep(); 
                timer++;
        }
        if(!triger_sensor){
            cout<<"##################################################"<<endl;
            cout<<"Unable to detect "<<sensor_name<<"  ##"<< i+1 << "##.  Quit "<<sensor_name<<" test" <<endl;
            cout<<"##################################################"<<endl<<endl; ;
            return false;
        }
    }

    return true;
}

bool irsTest(int index){
    int timer = 0, range = 3;
    bool triger_sensor = false;
    string sensor_name = "infrared";
    if(index!=0){
        range = index;
    }
    else{
        index = 1;
    }

    for(int i=index-1; i<range; i++){
        timer = 0;
        triger_sensor = false;
        while(ros::ok() && timer<TIME_OUT){
            ros::spinOnce(); 
            cout<<endl;
            cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<"  ##"<< i+1 <<"##  within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
            cout<<"Current "<<sensor_name<<" reading: 1(left): "<<ir_data[0]<<", 2(rear): "<<ir_data[1]<<", 3(right): "<<ir_data[2]<<endl;  
            
            //ir sinal value 3 indicates both emitter signals are received
            if(ir_data[i] == 3){
                cout<<endl;
                SetRobot.setSound(1,1);
                triger_sensor = true;
                cout<<"##################################################"<<endl;
                cout<<"Detected "<<sensor_name<<"  ##"<< i+1 << "##  wtih value  ##"<< ir_data[i]<<"##"<<endl; 
                cout<<"##################################################"<<endl<<endl; 
                ros::Duration(DELAY).sleep(); 
                break;
            }
                ros::Duration(1.0).sleep(); 
                timer++;
        }
        if(!triger_sensor){
            cout<<"##################################################"<<endl;
            cout<<"Unable to detect "<<sensor_name<<"  ##"<< i+1 << "##.  Quit "<<sensor_name<<" test" <<endl;
            cout<<"##################################################"<<endl<<endl; ;
            return false;
        }
    }

    return true;
}

bool cliffsTest(int index){
    int timer = 0, range = 4, test_dis = 50;
    bool triger_sensor = false;
    string sensor_name = "cliff";
    if(index!=0){
        range = index;
    }
    else{
        index = 1;
    }

    for(int i=index-1; i<range; i++){
        test_dis = 5;
        timer = 0;
        triger_sensor = false;
        while(ros::ok() && timer<TIME_OUT){
            ros::spinOnce(); 
            cout<<endl;
            if(test_dis==5)
                cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<"  ##"<< i+1 <<"##  less than "<< test_dis<< "cm within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
            else
                cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<"  ##"<< i+1 <<"##  larger than "<< test_dis<< "cm within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
            cout<<"Current "<<sensor_name<<" reading: 1: "<<cliff_data[0]<<", 2: "<<cliff_data[1]<<", 3: "<<cliff_data[2]<<", 4: "<<cliff_data[3]<<endl;  
            
            //sonar value 0 means out of range
            if(cliff_data[i] > 0){
                if((cliff_data[i]<test_dis && test_dis==5) || (cliff_data[i]>test_dis && test_dis==20)){
                    cout<<endl;
                    SetRobot.setSound(1,1);
                    triger_sensor = true;
                    cout<<"##################################################"<<endl;
                    cout<<"Detected "<<sensor_name<<"  ##"<< i+1 << "##  wtih value  ##"<< cliff_data[i]<<"##"<<endl; 
                    cout<<"##################################################"<<endl<<endl; 
                    ros::Duration(DELAY).sleep(); 
                    if(test_dis==5){
                        timer=0;
                        triger_sensor = false;
                        test_dis = 20;
                    }
                    else{
                        break;
                    }
                }
            }
                ros::Duration(1.0).sleep(); 
                timer++;
        }
        if(!triger_sensor){
            cout<<"##################################################"<<endl;
            cout<<"Unable to detect "<<sensor_name<<"  ##"<< i+1 << "##.  Quit "<<sensor_name<<" test" <<endl;
            cout<<"##################################################"<<endl<<endl; ;
            return false;
        }
    }

    return true;
}

bool proxisTest(int index){
    int timer = 0, range = 2;
    bool triger_sensor = false;
    string sensor_name = "proximity";
    if(index!=0){
        range = index;
    }
    else{
        index = 1;
    }

    for(int i=index-1; i<range; i++){
        timer = 0;
        triger_sensor = false;
        while(ros::ok() && timer<TIME_OUT){
            ros::spinOnce(); 
            cout<<endl;
            cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<"  ##"<< i+1 <<"##  within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
            cout<<"Current "<<sensor_name<<" reading: 1: "<<proxi_data[0]<<", 2: "<<proxi_data[1]<<endl;  
            
            //proximity sensor value 1 means detect obstacle
            if(proxi_data[i] == 1){
                cout<<endl;
                SetRobot.setSound(1,1);
                triger_sensor = true;
                cout<<"##################################################"<<endl;
                cout<<"Detected "<<sensor_name<<"  ##"<< i+1 << "##  wtih value  ##"<< proxi_data[i]<<"##"<<endl; 
                cout<<"##################################################"<<endl<<endl; 
                ros::Duration(DELAY).sleep(); 
                break;
            }
                ros::Duration(1.0).sleep(); 
                timer++;
        }
        if(!triger_sensor){
            cout<<"##################################################"<<endl;
            cout<<"Unable to detect "<<sensor_name<<"  ##"<< i+1 << "##.  Quit "<<sensor_name<<" test" <<endl;
            cout<<"##################################################"<<endl<<endl; ;
            return false;
        }
    }

    return true;
}

bool loadTest(){
    int timer = 0, test_load = 20;
    bool triger_sensor = false;
    string sensor_name = "load";

    while(ros::ok() && timer<TIME_OUT){
        ros::spinOnce(); 
        cout<<endl;
        cout<<"Testing "<<sensor_name<<". Please triger "<<sensor_name<<" in  ##["<<test_load<<","<<2*test_load<<"]kg##  within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
        cout<<"Current "<<sensor_name<<" reading: "<<load_data<<endl;  
        
        if(load_data>test_load && load_data<2*test_load){
            cout<<endl;
            SetRobot.setSound(1,1);
            cout<<"##################################################"<<endl;
            cout<<"Detected "<<sensor_name<<" with value  ##"<<load_data<<"##"<<endl;
            cout<<"##################################################"<<endl<<endl;
            ros::Duration(DELAY).sleep(); 
            if(test_load==20){
                timer = 0;
                test_load = 40;
            }
            else{
                triger_sensor = true;
                break;
            }
        }
        ros::Duration(1.0).sleep(); 
        timer++;
    }

    if(!triger_sensor){
        cout<<"##################################################"<<endl;
        cout<<"Unable to detect "<<sensor_name<<".  Quit "<<sensor_name<<" test" <<endl;
        cout<<"##################################################"<<endl<<endl;
        return false;
    }
 
    return true;
}

bool batteryTest(){
    int timer = 0;
    bool triger_sensor = false;
    string sensor_name = "battery_charging";

    while(ros::ok() && timer<TIME_OUT){
        ros::spinOnce(); 
        cout<<endl;
        cout<<"Testing "<<sensor_name<<". Please charging "<<sensor_name<<" within  ##"<<TIME_OUT-timer<<"##  sec..."<<endl;
        cout<<"Current "<<sensor_name<<" reading: "<<battery_charging<<endl;  
        
        //bumper value 0 means collision
        if(battery_charging){
            cout<<endl;
            SetRobot.setSound(1,1);
            triger_sensor = true;
            cout<<"##################################################"<<endl;
            cout<<"Detected "<<sensor_name<<" with value  ##CHARGING##"<<endl;
            cout<<"##################################################"<<endl<<endl;
            ros::Duration(DELAY).sleep(); 
            break;
        }
        ros::Duration(1.0).sleep(); 
        timer++;
    }

    if(!triger_sensor){
        cout<<"##################################################"<<endl;
        cout<<"Unable to detect "<<sensor_name<<".  Quit "<<sensor_name<<" test" <<endl;
        cout<<"##################################################"<<endl<<endl;
        return false;
    }
 
    return true;
}

bool motorTest(){ 
    int timer = 0;
    string cmd = "gnome-terminal -x bash -c \"source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;rosrun teleop_twist_keyboard teleop_twist_keyboard.py\"";
    system(cmd.c_str());
    enable_motor = true;
    cout<<"####################MOTOR TEST####################"<<endl;
    cout<<"You have  ##"<<TIME_OUT<<"##  sec to move the robot."<<endl;
    cout<<"Keyboard for move robot:"<<endl;
    cout<<"Forward:    8"<<endl;
    cout<<"Backward:   2"<<endl;
    cout<<"Turn right: 6"<<endl;
    cout<<"Turn left:  4"<<endl;
    cout<<"Stop:       5"<<endl;
    cout<<"##################################################"<<endl<<endl;
    while(ros::ok() && timer<TIME_OUT){
        ros::spinOnce();
        SetRobot.setMotorSpeed(right_vel<0 ? 'B' : 'F',abs(right_vel),left_vel<0 ? 'B' : 'F',abs(left_vel));
        ros::Duration(1.0).sleep(); 
        timer++;
    }
    cmd ="rosnode kill /teleop_twist_keyboard";
    system(cmd.c_str());
    enable_motor = false;
    left_vel = 0;
    right_vel = 0;
    SetRobot.setMotorSpeed('F', 0, 'F', 0);
    return true;
}


void printLog(){
    cout<<"####################LOG HISTORY###################"<<endl;
    cout<<"COMMAND CHECKLIST:"<<endl;
    cout<<"\"bumpers\" or  \"1\"   for bumper test"<<endl;
    cout<<"\"sonars\"  or  \"2\"   for sonar test"<<endl;
    cout<<"\"irs\"     or  \"3\"   for infrared test"<<endl;
    cout<<"\"cliffs\"  or  \"4\"   for cliff test"<<endl;
    cout<<"\"proxis\"  or  \"5\"   for proximity test"<<endl;
    cout<<"\"load\"    or  \"6\"   for load test"<<endl;
    cout<<"\"battery\" or  \"7\"   for battery test"<<endl;
    cout<<"\"motor\"   or  \'9\'   for motor test"<<endl;
    cout<<endl;
    cout<<"COMMAND HISTORY:"<<endl;
    if(test_log.size()==0){
        cout<<"There is no test history!"<<endl;
    }
    else{
        for(int i=0; i<test_log.size();i++){
            cout<<i+1<<": "<<test_log.at(i)<<endl;
        }
    }
    cout<<"##################################################"<<endl<<endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "hardware_check");
    ros::NodeHandle n;

    SetRobot.initialize();
    
    //Startup begin
    //sleep for 1 second, otherwise waitForService not work properly
    ros::Duration(1.0).sleep();
    ROS_INFO("(startup) Waiting for Robot setting hardware...");
    ros::service::waitForService("/gobot_startup/sensors_ready", ros::Duration(60.0));
    ROS_INFO("(startup) Robot setting hardware is ready.");
    //Startup end

    ros::Subscriber bumpersSub = n.subscribe("/gobot_base/bumpers_raw_topic", 1, bumperCallback);
    ros::Subscriber sonarSub = n.subscribe("/gobot_base/sonar_topic", 1, sonarCallback);
    ros::Subscriber irSub = n.subscribe("/gobot_base/ir_topic", 1, irCallback);
    ros::Subscriber cliffSub = n.subscribe("/gobot_base/cliff_topic", 1, cliffCallback);
    ros::Subscriber proxiSub = n.subscribe("/gobot_base/proximity_topic", 1, proxiCallback);
    ros::Subscriber loadSub = n.subscribe("/gobot_base/weight_topic", 1, loadCallback);
    ros::Subscriber batterySub = n.subscribe("/gobot_base/battery_topic", 1, batteryCallback);
    ros::Subscriber velSub = n.subscribe("cmd_vel", 1, velCallback);


    string input_str = "", index_str = "", result = "failed";;
    int index = 0;
    char command[99], index_char;
    ros::Duration(DELAY).sleep(); 
    while(ros::ok()){
        SetRobot.setLed(1,{"white","blue"});
        cout<<"####################START TEST####################"<<endl;
        cout<<"Start robot hardware check..."<<endl;
        cout<<"COMMAND CHECKLIST:"<<endl;
        cout<<"\"exit\"    or  \"0\"   for exit test"<<endl;
        cout<<"\"bumpers\" or  \"1\"   for bumper test"<<endl;
        cout<<"\"sonars\"  or  \"2\"   for sonar test"<<endl;
        cout<<"\"irs\"     or  \"3\"   for infrared test"<<endl;
        cout<<"\"cliffs\"  or  \"4\"   for cliff test"<<endl;
        cout<<"\"proxis\"  or  \"5\"   for proximity test"<<endl;
        cout<<"\"load\"    or  \"6\"   for load test"<<endl;
        cout<<"\"battery\" or  \"7\"   for battery test"<<endl;
        cout<<"\"motor\"   or  \'9\'   for motor test"<<endl;
        cout<<"\"log\"     or  \"99\"  for view test log"<<endl;
        cout<<endl;
        cout<<"Please input your choice: ";
        cin>>command;
        input_str = command;
        cout<<"Your input is:  ##"<<input_str<<"##"<<endl;
        cout<<"Test is going to start after 1.0 sec..."<<endl;
        SetRobot.setSound(1,1);
        cout<<"##################################################"<<endl<<endl;
        ros::Duration(1.0).sleep();
        //if input is exit, end the while loop to complete test
        if(input_str.compare("exit")==0 || input_str.compare("0")==0)
            break;
        

        if(input_str.compare("bumpers")==0 || input_str.compare("1")==0){
            cout<<"##################SELECT INDEX####################"<<endl;
            cout<<"Select which bumper is going to be tested: valid range[1,8]"<<endl;
            cout<<"Other inputs will test all bumpers"<<endl;
            cout<<endl;
            cout<<"Please input your slection: ";
            cin>>index_char;

            if(index_char>='1' && index_char<='8'){
                index_str = index_char;
                index = stoi(index_str);
                cout<<"Bumper  ##"<<index<<"##  will be tested"<<endl;
            }
            else{
                index = 0;
                cout<<"All bumpers will be tested"<<endl;
            }
            cout<<"##################################################"<<endl<<endl;
            SetRobot.setLed(1,{"white","blue"});
            ros::Duration(1.0).sleep();
            
            if(bumpersTest(index)){
                result = "successful";
            }
        }

        else if(input_str.compare("sonars")==0 || input_str.compare("2")==0){
            cout<<"##################SELECT INDEX####################"<<endl;
            cout<<"Select which sonar is going to be tested: valid range[1,4]"<<endl;
            cout<<"Other inputs will test all sonars"<<endl;
            cout<<endl;
            cout<<"Please input your slection: ";
            cin>>index_char;

            if(index_char>='1' && index_char<='4'){
                index_str = index_char;
                index = stoi(index_str);
                cout<<"Sonar  ##"<<index<<"##  will be tested"<<endl;
            }
            else{
                index = 0;
                cout<<"All sonars will be tested"<<endl;
            }
            cout<<"##################################################"<<endl<<endl;
            SetRobot.setLed(1,{"white","green"});
            ros::Duration(1.0).sleep();

            if(sonarsTest(index)){
                result = "successful";
            }
        }

        else if(input_str.compare("irs")==0 || input_str.compare("3")==0){
            cout<<"##################SELECT INDEX####################"<<endl;
            cout<<"Select which ir is going to be tested: valid range[1,3]"<<endl;
            cout<<"Other inputs will test all irs"<<endl;
            cout<<endl;
            cout<<"Please input your slection: ";
            cin>>index_char;

            if(index_char>='1' && index_char<='3'){
                index_str = index_char;
                index = stoi(index_str);
                cout<<"Ir  ##"<<index<<"##  will be tested"<<endl;
            }
            else{
                index = 0;
                cout<<"All irs will be tested"<<endl;
            }
            cout<<"##################################################"<<endl<<endl;
            SetRobot.setLed(1,{"white","green"});
            ros::Duration(1.0).sleep();

            if(irsTest(index)){
                result = "successful";
            }
        }

        else if(input_str.compare("cliffs")==0 || input_str.compare("4")==0){
            cout<<"##################SELECT INDEX####################"<<endl;
            cout<<"Select which cliff is going to be tested: valid range[1,4]"<<endl;
            cout<<"Other inputs will test all cliffs"<<endl;
            cout<<endl;
            cout<<"Please input your slection: ";
            cin>>index_char;

            if(index_char>='1' && index_char<='4'){
                index_str = index_char;
                index = stoi(index_str);
                cout<<"Cliff  ##"<<index<<"##  will be tested"<<endl;
            }
            else{
                index = 0;
                cout<<"All cliffs will be tested"<<endl;
            }
            cout<<"##################################################"<<endl<<endl;
            SetRobot.setLed(1,{"white","green"});
            ros::Duration(1.0).sleep();

            if(cliffsTest(index)){
                result = "successful";
            }
        }

        else if(input_str.compare("proxis")==0 || input_str.compare("5")==0){
            cout<<"##################SELECT INDEX####################"<<endl;
            cout<<"Select which proximity is going to be tested: valid range[1,2]"<<endl;
            cout<<"Other inputs will test all proximitys"<<endl;
            cout<<endl;
            cout<<"Please input your slection: ";
            cin>>index_char;

            if(index_char>='1' && index_char<='2'){
                index_str = index_char;
                index = stoi(index_str);
                cout<<"Proximity  ##"<<index<<"##  will be tested"<<endl;
            }
            else{
                index = 0;
                cout<<"All proximitys will be tested"<<endl;
            }
            cout<<"##################################################"<<endl<<endl;
            SetRobot.setLed(1,{"white","green"});
            ros::Duration(1.0).sleep();

            if(proxisTest(index)){
                result = "successful";
            }
        }

        else if(input_str.compare("load")==0 || input_str.compare("6")==0){
            SetRobot.setLed(1,{"white","green"});
            if(loadTest()){
                result = "successful";
            }
        }

        else if(input_str.compare("battery")==0 || input_str.compare("7")==0){
            SetRobot.setLed(1,{"white","green"});
            if(batteryTest()){
                result = "successful";
            }
        }

        else if(input_str.compare("motor")==0 || input_str.compare("9")==0){
            SetRobot.setLed(1,{"white","green"});
            if(motorTest()){
                result = "successful";
            }
        }

        else if(input_str.compare("log")==0 || input_str.compare("99")==0){
            SetRobot.setLed(1,{"white","green"});
            printLog();
            result = "successful";
        }

        else{
            cout<<"####################INVALID#######################"<<endl;
            cout<<"Invalid command received! ("<<input_str<<")"<<endl;
            cout<<"##################################################"<<endl<<endl;
            input_str = "invalid";
        }

        if(input_str.compare("invalid")!=0){
            //set robot led and sound to indicate result -> successful or failed
            if(result.compare("successful")==0){
                SetRobot.setLed(0,{"green"});
                SetRobot.setSound(1,2);
            }
            else{
                SetRobot.setLed(0,{"red"});
                SetRobot.setSound(3,2);
            }

            //record the test history, excluding "log" command itself
            if(input_str.compare("log")!=0 && input_str.compare("99")!=0){
                test_log.push_back("Input: "+ input_str + "  ||  Index: " + to_string(index) + "  ||  Result: " + result );

                cout<<"#####################TEST RESULT##################"<<endl;
                cout<<"End of  ##"<<input_str<<"##  test."<<endl;
                cout<<"Result of  ##"<<input_str<<"##  :  ##"<< result <<"##"<<endl;
                cout<<endl;
                cout<<"Please proceed to next test after "<<DELAY<<" sec..."<<endl;
                cout<<"##################################################"<<endl<<endl;
            }

            ros::Duration(DELAY).sleep();
        }

        //reset params for next test
        input_str = "";
        index = 0;
        result = "failed";
        ros::spinOnce();
    }

    SetRobot.setLed(0,{"white"});
    return 0;
}