/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "open_manipulator_core/debug_pos_controller.h"
#include "open_manipulator_core/DynamixelDebug.h"
#include <vector>


DebugPositionControl::DebugPositionControl()
    :node_handle_("")
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint8_t scan_range        = node_handle_.param<int>("scan_range", 200);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
  
  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
  {
    ROS_ERROR("Not found Motors, Please check scan range or baud rate");
    ros::shutdown();
    return;
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Position");

  initPublisher();
  initSubscriber();
  initServer();
}

DebugPositionControl::~DebugPositionControl()
{
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void DebugPositionControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  dynamixel_workbench controller; position control example             \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void DebugPositionControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
  debug_data_pub_ = node_handle_.advertise<open_manipulator_core::DynamixelDebug>("debug_data",10);
}

void DebugPositionControl::initSubscriber()
{
  joint_command_sub_ = node_handle_.subscribe("goal_dynamixel_position", 10, &DebugPositionControl::goalJointPositionCallback, this);
}

void DebugPositionControl::initServer()
{
  joint_command_server_ = node_handle_.advertiseService("joint_command", &DebugPositionControl::jointCommandMsgCallback, this);
}

void DebugPositionControl::dynamixelStatePublish()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id                  = dxl_id_[index];
    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void DebugPositionControl::jointStatePublish()
{
  int32_t present_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

  int32_t present_velocity[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
    present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

  sensor_msgs::JointState dynamixel_;
  dynamixel_.header.stamp = ros::Time::now();

  for (int index = 0; index < dxl_cnt_; index++)
  {
    std::stringstream id_num;
    id_num << "id_" << (int)(dxl_id_[index]);

    dynamixel_.name.push_back(id_num.str());

    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]));
    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]));
  }
  joint_states_pub_.publish(dynamixel_);
}

void DebugPositionControl::debugDataPublish(){
   std::vector<int32_t> ids;
   std::vector<int32_t> temps;
   std::vector<int32_t> load;
   std::vector<int32_t> volt;
   std::vector<int32_t> current;
   std::vector<int32_t> presentPos;
   std::vector<int32_t> presentVel;
   std::vector<int32_t> goalPos;
   std::vector<int32_t> goalVel;
   std::vector<int32_t> RDT;
   std::vector<int32_t> FF1stGain;
   std::vector<int32_t> FF2ndGain;
   std::vector<int32_t> errorStatus;
   std::vector<int32_t> tempLimit;
   std::vector<int32_t> posPGain;
   std::vector<int32_t> posIGain;
   std::vector<int32_t> posDGain;
   std::vector<int32_t> velPGain;
   std::vector<int32_t> velIGain;
           
   for (int index = 0; index < dxl_cnt_; index++){
      ids.push_back(dxl_id_[index]);
      temps.push_back(dxl_wb_->itemRead(dxl_id_[index], "Present_Temperature"));
      load.push_back(dxl_wb_->itemRead(dxl_id_[index], "Present_Load"));
      volt.push_back(dxl_wb_->itemRead(dxl_id_[index], "Present_Input_Volt"));
      current.push_back(dxl_wb_->itemRead(dxl_id_[index], "Present_Current"));
      presentPos.push_back(dxl_wb_->itemRead(dxl_id_[index], "Present_Position"));
      presentVel.push_back(dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity"));
      goalPos.push_back(dxl_wb_->itemRead(dxl_id_[index], "Goal_Position"));
      goalVel.push_back(dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity"));
      RDT.push_back(dxl_wb_->itemRead(dxl_id_[index], "Return_Delay_Time"));
      FF1stGain.push_back(dxl_wb_->itemRead(dxl_id_[index], "Feedforward_1st_Gain"));
      FF2ndGain.push_back(dxl_wb_->itemRead(dxl_id_[index], "Feedforward_2nd_Gain"));
      errorStatus.push_back(dxl_wb_->itemRead(dxl_id_[index], "Hardware_Error_Status"));
      tempLimit.push_back(dxl_wb_->itemRead(dxl_id_[index], "Temperature_Limit"));
      posPGain.push_back(dxl_wb_->itemRead(dxl_id_[index], "Position_P_Gain"));
      posIGain.push_back(dxl_wb_->itemRead(dxl_id_[index], "Position_I_Gain"));
      posDGain.push_back(dxl_wb_->itemRead(dxl_id_[index], "Position_D_Gain"));
      velPGain.push_back(dxl_wb_->itemRead(dxl_id_[index], "Velocity_P_Gain"));
      velIGain.push_back(dxl_wb_->itemRead(dxl_id_[index], "Velocity_I_Gain"));      
   }
   open_manipulator_core::DynamixelDebug debug_data;
   
   debug_data.dxl_id = ids;
   debug_data.present_temp = temps;
   debug_data.present_load = load;
   debug_data.present_volt = volt;
   debug_data.present_current = current;
   debug_data.present_pos = presentPos;
   debug_data.present_vel = presentVel;
   debug_data.goal_pos = goalPos;
   debug_data.goal_vel = goalVel;
   debug_data.return_delay_time = RDT;
   debug_data.feedforward_1st_gain = FF1stGain;
   debug_data.feedforward_2nd_gain = FF2ndGain;
   debug_data.error_status = errorStatus;
   debug_data.temp_limit = tempLimit;
   debug_data.pos_p_gain = posPGain;
   debug_data.pos_i_gain = posIGain;
   debug_data.pos_d_gain = posDGain;
   debug_data.vel_p_gain = velPGain;
   debug_data.vel_i_gain = velIGain;      
   
   debug_data_pub_.publish(debug_data);
}

void DebugPositionControl::controlLoop()
{
  dynamixelStatePublish();
  jointStatePublish();
  debugDataPublish();
}

bool DebugPositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  int32_t goal_position = 0;
  int32_t present_position = 0;

  if (req.unit == "rad")
  {
    goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
  }
  else if (req.unit == "raw")
  {
    goal_position = req.goal_position;
  }
  else
  {
    goal_position = req.goal_position;
  }

  bool ret = dxl_wb_->goalPosition(req.id, goal_position);

  res.result = ret;
}

void DebugPositionControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double goal_position[dxl_cnt_] = {0.0, };

  for (int index = 0; index < dxl_cnt_; index++)
    goal_position[index] = msg->position.at(index);

  int32_t goal_dxl_position[dxl_cnt_] = {0, };

  for (int index = 0; index < dxl_cnt_; index++)
  {
    goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], goal_position[index]);
  }

  dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "debug_position_control");
  DebugPositionControl pos_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
