#include <gobot_recovery/startup.h>

ros::Publisher vel_pub;
std_srvs::Empty empty_srv;

int count = 1;
bool buttonOn=true;
ros::Time action_time;
std::string restartRobotFile;

//-1=no goal, 0=complete goal, 1=active goal
int goal_state = -1;
bool robot_pause = false;

ros::Publisher goalCancel_pub;
void initialPoseResultCallback(const std_msgs::Int8::ConstPtr& msg){
  if(msg->data==0){
    ROS_ERROR("Can not find robot pose when start up. Retry after 10 sec.");
    ROS_ERROR("Or move robot to charging station and restart.");
    if(count<3){
      ROS_ERROR("Tried %d times",count);
      ros::Duration(10.0).sleep();
      while(!ros::service::call("/gobot_recovery/globalize_pose",empty_srv)){
          ROS_ERROR("Failed to start robot");
          ros::service::call("/gobot_recovery/stop_globalize_pose",empty_srv);
          ros::Duration(1.0).sleep();
      }
      count++;
    }
    else{
      ROS_ERROR("Give up! Move robot to charging station and restart.");
    }
  }
}


void getButtonCallback(const std_msgs::Int8::ConstPtr& msg){
  if(msg->data==0 && buttonOn){
		action_time = ros::Time::now();
		buttonOn=false;
	}
	else if(msg->data==1 && !buttonOn){
		buttonOn = true;
    double dt = (ros::Time::now() - action_time).toSec();
    if(dt>1.0 && dt<5.0 && goal_state==1){
      if(robot_pause){
        ROS_INFO("Continue robot.");
        ros::service::call("/gobot_base/continue_robot",empty_srv);
      }
      else{
        ROS_INFO("Pause robot.");
        ros::service::call("/gobot_base/pause_robot",empty_srv);
      }
      robot_pause = !robot_pause;
    }
    else if(dt>15.0){
      ros::service::call("/gobot_recovery/go_home",empty_srv);
		}
		else if(dt>999.0){
			//restart
      ROS_INFO("Restart robot. Please wait for xx seconds...");
			const std::string restart_script = "sudo sh " + restartRobotFile + " &";
      system(restart_script.c_str());
		}
	}
}

void goalGetCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg){
  goal_state=1;
}

void goalResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  goal_state=0;
}

void mySigintHandler(int sig)
{

  ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "startup");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    ROS_INFO("Starting Robot...");
    ros::Duration(5.0).sleep();

    nh.getParam("restart_robot_file", restartRobotFile);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    goalCancel_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);

    ros::Subscriber goalResult = nh.subscribe("/move_base/result",1,goalResultCallback);
    ros::Subscriber goalGet = nh.subscribe("/move_base/goal",1,goalGetCallback);
    ros::Subscriber initialPoseResult = nh.subscribe("/gobot_recovery/find_initial_pose",1, initialPoseResultCallback);
    ros::Subscriber button_sub = nh.subscribe("button_topic",1,getButtonCallback);
    
    ros::service::waitForService("/gobot_recovery/initialize_pose", ros::Duration(30.0));
    while(!ros::service::call("/gobot_recovery/initialize_pose",empty_srv)){
        ROS_ERROR("Failed to initilize robot pose");
        ros::service::call("/gobot_recovery/stop_globalize_pose",empty_srv);
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Started Robot.");

    ros::spin();
    return 0;
}