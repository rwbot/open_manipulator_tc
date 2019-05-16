#ifndef _GRIPPER_COMMANDER_H_
#define _GRIPPER_COMMANDER_H_


#include <ros/ros.h>

#include <std_msgs/String.h>

class GripperCommander {
   private:
      ros::NodeHandle nh_;
      
      ros::Subscriber gripper_command_sub;
      ros::ServiceClient joint_command_client;
      
   public:
      GripperCommander();
      ~GripperCommander();
      
      void onCommand(const std_msgs::String::ConstPtr& msg);
};


#endif //define _GRIPPER_COMMANDER_H_
