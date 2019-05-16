#include <ros/ros.h>

#include "dynamixel_workbench_msgs/AX.h"

#include "dynamixel_workbench_msgs/RX.h"

#include "dynamixel_workbench_msgs/MX.h"
#include "dynamixel_workbench_msgs/MXExt.h"
#include "dynamixel_workbench_msgs/MX2.h"
#include "dynamixel_workbench_msgs/MX2Ext.h"

#include "dynamixel_workbench_msgs/EX.h"

#include "dynamixel_workbench_msgs/XL320.h"
#include "dynamixel_workbench_msgs/XL.h"

#include "dynamixel_workbench_msgs/XM.h"
#include "dynamixel_workbench_msgs/XMExt.h"

#include "dynamixel_workbench_msgs/XH.h"

#include "dynamixel_workbench_msgs/PRO.h"
#include "dynamixel_workbench_msgs/PROExt.h"

#include <sensor_msgs/JointState.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

int main(int argc, char** argv) {

   ros::init(argc, argv, "pid_setup");
   ros::NodeHandle nh_("");

   std::string device_name = nh_.param<std::string>("device_name", "/dev/ttyUSB0");
   uint32_t dxl_baud_rate = nh_.param<int>("baud_rate", 3000000);
   
   uint8_t scan_range = nh_.param<int>("scan_range", 200);
   
   DynamixelWorkbench *dxl_wb_ = new DynamixelWorkbench;
   
   dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);
   
   uint8_t dxl_cnt_;
   uint8_t dxl_id_[16];
   if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true) {
      ROS_ERROR("Not found Motors, Please check scan range or baud rate");
      ros::shutdown();
      return 1;
   }
   
   int32_t p_gain = 1000;
   int32_t i_gain = 200;
   int32_t d_gain = 500;
   
   printf("Count: %d\n", dxl_cnt_);
   for (int i = 0; i < dxl_cnt_; i++) {
      printf("Dynamixel: %d\n", dxl_id_[i]);
      dxl_wb_->itemWrite(dxl_id_[i], "Position_P_Gain", p_gain);
      dxl_wb_->itemWrite(dxl_id_[i], "Position_I_Gain", i_gain);
      dxl_wb_->itemWrite(dxl_id_[i], "Position_D_Gain", d_gain);
      p_gain = dxl_wb_->itemRead(dxl_id_[i], "Position_P_Gain");
      i_gain = dxl_wb_->itemRead(dxl_id_[i], "Position_I_Gain");
      d_gain = dxl_wb_->itemRead(dxl_id_[i], "Position_D_Gain");
      printf("\tP: %d\n", p_gain);
      printf("\tI: %d\n", i_gain);
      printf("\tD: %d\n", d_gain);
   }
   
   ros::shutdown();
   return 0;
}
