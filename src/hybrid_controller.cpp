// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_more_controllers/hybrid_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "pseudo_inversion.h"

namespace franka_more_controllers {
Eigen::Matrix<double,6,1> f_measured_hybrid;

bool HybridController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &HybridController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_forcetorque_sensor = node_handle.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, &HybridController::updateFTsensor, this,ros::TransportHints().reliable().tcpNoDelay());
  publisher = node_handle.advertise<std_msgs::Float64MultiArray>("position_force", 1000);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("HybridController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "HybridController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }
  // Get axis and desired_force value from launch file
  if (!node_handle.getParam("/applied_axis", which_axis)){
      ROS_ERROR("Could not find applied_axis parameter");
      return false;
  }
  if (which_axis.compare("x") == 0){axis = 0;}
  else if (which_axis.compare("y") == 0){axis = 1;}
  else if (which_axis.compare("z") == 0){axis = 2;}
  else if (which_axis.compare("tau_x") == 0){axis = 3;}
  else if (which_axis.compare("tau_y") == 0){axis = 4;}
  else if (which_axis.compare("tau_z") == 0){axis = 5;}
  if (!node_handle.getParam("/FT_desired", desired_force_torque_value)){
      ROS_ERROR("Could not find FT_desired parameter");
      return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "HybridController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "HybridController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("HybridController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "HybridController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "HybridController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "HybridController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_more_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&HybridController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void HybridController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigenforce_controller_invdynamics
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  err_force_int.setZero(); //reset integration term

  q_d_nullspace_ = q_initial;  // set nullspace equilibrium configuration to initial q
}
void HybridController::updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  geometry_msgs::Wrench f_meas = msg->wrench;
	// f_cur_buffer_ = f_meas.force.x;
  f_measured_hybrid(0) = f_meas.force.x;
  f_measured_hybrid(1) = f_meas.force.y;
  f_measured_hybrid(2) = f_meas.force.z;
  f_measured_hybrid(3) = f_meas.torque.x;
  f_measured_hybrid(4) = f_meas.torque.y;
  f_measured_hybrid(5) = f_meas.torque.z;
}
void HybridController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1> > force_ext(robot_state.O_F_ext_hat_K.data());
  Eigen::Matrix<double, 6, 1>  cart_velocity;

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // FORCE CONTROL
  Eigen::Matrix<double, 6,6> kp_force,ki_force, Sf, Id, Sp;
  kp_force.setIdentity();
  ki_force.setIdentity();
  kp_force *= 0.11;
  ki_force *= 0.14;
  Sf.setZero();
  Sf(axis,axis) = 1; //Selection matrix for force (only in z-axis)
  Id.setIdentity();
  Sp = Id - Sf; //Selection matrix for position

  Eigen:: Matrix<double,6,1> desired_force_torque, err_force;
  desired_force_torque.setZero();
  desired_force_torque(axis) = desired_force_torque_value;

  err_force = desired_force_torque - f_measured_hybrid;
  err_force_int += err_force*period.toSec(); //integral term of force error

  Eigen::VectorXd force_ctrl(6);
  force_ctrl = kp_force*err_force + ki_force*err_force_int;
  // position_d_(axis) += force_ctrl(axis);   // compute error to desired pose
  if ((abs(desired_force_torque(axis) - f_measured_hybrid(axis)) < 2.5) && (position(0) <= 0.5)){ //When contact happens
    position_d_(0) += 0.00001; // 0.00001
    std::cout << "x new: " << position_d_(0)  << std::endl;
  }
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;  // position error
  std::cout << "error in " << which_axis <<": " << err_force(axis) << std::endl;

  cart_velocity = jacobian * dq;
  std_msgs::Float64MultiArray msg;
  msg.data.clear();
  msg.data.push_back(position(0));
  msg.data.push_back(position(1));
  msg.data.push_back(position(2));
  msg.data.push_back(f_measured_hybrid(0));
  msg.data.push_back(f_measured_hybrid(2));
  publisher.publish(msg);
  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control+ Sf * force_ctrl
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7),tau_force(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  tau_force << jacobian.transpose() * (Sf * force_ctrl);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (Sp * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq)));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis + tau_force;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  // std::cout << tau_d.transpose() << std::endl;

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;

  desired_force_torque_value = filter_params_ * target_desired_force_torque_value + (1 - filter_params_) * desired_force_torque_value;

  // position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  // Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  // Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
  // aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() +
  //                           (1.0 - filter_params_) * aa_orientation_d.axis();
  // aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() +
  //                            (1.0 - filter_params_) * aa_orientation_d.angle();
  // orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
}

Eigen::Matrix<double, 7, 1> HybridController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void HybridController::complianceParamCallback(
    franka_more_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_(axis,axis) = 20.0;
  cartesian_stiffness_target_(0,0) = 275.0; //150

  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  target_desired_force_torque_value = config.desired_force_torque;

}

void HybridController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_more_controllers

PLUGINLIB_EXPORT_CLASS(franka_more_controllers::HybridController,
                       controller_interface::ControllerBase)
