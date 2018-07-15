#ifndef ROBOT_MOVE_CLASS
#define ROBOT_MOVE_CLASS

//ros headers
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>

//c++ headers
#include <vector>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <thread>
#include <string>

#include <gobot_msg_srv/set_robot_class.h>

namespace robot_class {

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  struct Point {
    double x;
    double y;
    double yaw;
  };

  class RobotMoveClass : public robot_class::SetRobot
  {
    public:
      RobotMoveClass();
      ~RobotMoveClass();

      void moveClientInitialize();

      void moveFromCS();

      void moveTo(Point point);
      void moveTo(double x, double y, double yaw);
      //tf::createQuaternionMsgFromYaw   //tf::getYaw (const geometry_msgs::Quaternion &msg_q)
      void moveTo(double x, double y, geometry_msgs::Quaternion quternion);
      void moveTo(move_base_msgs::MoveBaseGoal point);

      void cancelMove();

      actionlib::SimpleClientGoalState getGoalState();

      void turnLeft(const int vel_r, const int vel_l);
      void turnRight(const int vel_r, const int vel_l);
      void turnLeft(const int vel);
      void turnRight(const int vel);
      void backward(const int vel);
      void forward(const int vel);

      void getStatus(int &status, std::string &text);

      void setAutoSpeedLimit(double linear_v, double angular_v);

      void setManualSpeedLimit(double linear_v, double angular_v);


    private:
      MoveBaseClient* move_client_;

  };
};


#endif