#include "dynamixel_trajectory_action/gripper_commander.h"

#include "dynamixel_workbench_msgs/JointCommand.h"


GripperCommander::GripperCommander() {
   gripper_command_sub = nh_.subscribe("/gripper_command", 1, &GripperCommander::onCommand, this);
   joint_command_client = nh_.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");
}

GripperCommander::~GripperCommander() {}


void GripperCommander::onCommand(const std_msgs::String::ConstPtr& msg) {

   dynamixel_workbench_msgs::JointCommand commandMsg;
   commandMsg.request.unit = "raw";
   commandMsg.request.id = 7; //Replace 7 with variable
   
   if (msg->data == "grip_off") {
      commandMsg.request.goal_position = 0;
      if (joint_command_client.call(commandMsg) && commandMsg.response.result) {
         ROS_INFO("grip set to \"off\"");
      }
   } else if (msg->data == "grip_on") {
      commandMsg.request.goal_position = 1450;
      if (joint_command_client.call(commandMsg) && commandMsg.response.result) {
         ROS_INFO("grip set to \"on\"");
      }
   } else if (msg->data == "neutral") {
      commandMsg.request.goal_position = 512;
      if (joint_command_client.call(commandMsg) && commandMsg.response.result) {
         ROS_INFO("grip set to \"neutral\"");
      }
   } else {
      ROS_ERROR("If you want to grip or release something, publish 'grip_on', 'grip_off' or 'neutral'");
   }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_commander");

  ROS_INFO("Starting gripper commander;");
  GripperCommander gripper_commander;

  ROS_INFO("Spinning...");
  ros::spin();

  return 0;
}
