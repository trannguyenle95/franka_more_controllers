# franka_more_controllers

Adoption to Noetic 12/7/2024. Things to be changed:
 - In the automated.launch file, change "position_joint_trajectory_controller" to "effort_joint_trajectory_controller"
 - In the automated.py file, change "position_joint_trajectory_controller" to "effort_joint_trajectory_controller"
 - franka_control.srv is deprecated -> Remove or find some other ways to set collision behaviour.
