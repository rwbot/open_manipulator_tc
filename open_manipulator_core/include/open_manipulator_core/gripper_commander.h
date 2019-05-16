#ifndef _GRIPPER_COMMANDER_H_
#define _GRIPPER_COMMANDER_H_


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

class GripperCommander {
   private:
      ros::NodeHandle nh_;
      
      ros::Subscriber gripper_command_sub;
      ros::Subscriber joint_state_sub;
      ros::ServiceClient joint_command_client;
      
      double lastGripperPos;
      ros::Time lastTime;
      double gripperThreshold;
      ros::Duration waitTime;
      
      bool gripping;
   public:
      GripperCommander();
      ~GripperCommander();
      
      void onCommand(const std_msgs::String::ConstPtr& msg);
      void onJointState(const sensor_msgs::JointStateConstPtr& msg);
};


#endif //define _GRIPPER_COMMANDER_H_
