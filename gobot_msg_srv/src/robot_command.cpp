#include <gobot_msg_srv/robot_command.h>


namespace robot_class {

    RobotCommand::RobotCommand():SetRobot(){
    }
    RobotCommand::~RobotCommand(){};

    void RobotCommand::getPose(Pose &pose){
        pose.x=1;
        pose.y=1;
        pose.theta=1;
    }
};

