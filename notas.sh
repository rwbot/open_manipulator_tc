reset;roslaunch open_manipulator_moveit_config moveit_planning_execution.launch sim:=false

rosrun open_manipulator_core platform_pick_and_place.py


rostopic echo /gripper_command
