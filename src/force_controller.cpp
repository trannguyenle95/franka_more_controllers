// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_more_controllers/force_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <Eigen/LU>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>

namespace franka_more_controllers {
Eigen::Matrix<double,6,1> f_current;
double delta_z1;
int cout=0;
bool ForceController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "ForceController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "ForceController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("ForceController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      // if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
      //   ROS_ERROR_STREAM(
      //       "ForceController: Robot is not in the expected starting position for "
      //       "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
      //       "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      //   return false;
      // }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "ForceController: Exception getting state handle: " << e.what());
    return false;
  }
  sub_forcetorque_sensor_real = node_handle.subscribe<geometry_msgs::WrenchStamped>("/franka_state_controller/F_ext", 1, &ForceController::updateFTsensor, this,ros::TransportHints().reliable().tcpNoDelay());
  pub_data = node_handle.advertise<std_msgs::Float64>("data", 1);
  return true;
}

void ForceController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
  err_force_int.setZero(); //reset integration term
  err_force.setZero();
  delta_z1 = 0;
}
void ForceController::updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  geometry_msgs::Wrench f_meas = msg->wrench;
	f_cur_buffer_ = f_meas.force.x;
  f_current(1) = f_meas.force.y;
  f_current(2) = f_meas.force.z;
  f_current(3) = f_meas.torque.x;
  f_current(4) = f_meas.torque.y;
  f_current(5) = f_meas.torque.z;
  f_current(0) = first_order_lowpass_filter();
}
void ForceController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  double radius = 0.2;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_z = radius * (std::cos(angle) - 1);
  Eigen::Matrix<double, 6,6> kp_force,ki_force;
  kp_force.setZero();
  ki_force.setZero();
  // kp_force(2,2)= 0.0001; //
  // ki_force(2,2)= 0.0000001; // works for z
  kp_force(0,0)= 0.00001; //00000001
  ki_force(0,0)= 0.000005; // 00001
  //// set the desired force and calculate the force error.
  Eigen:: Matrix<double,6,1> f_desired;
  f_desired << 5,0,0,0,0,0; //force desired in z axis

  err_force = f_desired - f_current;
  err_force_int += err_force*elapsed_time_.toSec(); //integral term of force error

 //// calculate new pos in global frame
  Eigen::VectorXd force_ctrl(6);
  force_ctrl = kp_force*err_force + ki_force*err_force_int;
  delta_z1 = force_ctrl[0];
  std::array<double, 16> pose_current = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  std::array<double, 16> pose_current_wrong = cartesian_pose_handle_->getRobotState().O_T_EE;

  // delta_z1 -= 0.0001;
  pose_current[12] = pose_current[12] + delta_z1 * 0.0001; //for x. 0.001 works smooth and fast
  // pose_current[14] = pose_current[14] + delta_z1 * 0.0001; //for z

  std_msgs::Float64 msg;
  msg.data = pose_current[12];
  pub_data.publish(msg);
  // std::cout << "pose x : " << pose_current[12] << "\t delta_x1 : " << delta_z1 << "\t force control : " << force_ctrl[0] << std::endl;
  // if (cout < 1000) {
    // std::cout << "current pose :" << pose_current[12]  << "\t current pose withoud d" << pose_current_wrong[12] << std::endl;
  //   cout++;
  // }

  cartesian_pose_handle_->setCommand(pose_current);
}
double ForceController::first_order_lowpass_filter()
{
    filt_ = (tau_ * filt_old_ + delta_time*f_cur_buffer_)/(tau_ + delta_time);
    filt_old_ = filt_;
    return filt_;
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_more_controllers::ForceController,
                       controller_interface::ControllerBase)
