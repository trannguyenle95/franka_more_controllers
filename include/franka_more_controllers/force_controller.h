// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_more_controllers {

class ForceController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  /** @brief A low pass filter to smooth the force feedback signal */
  double first_order_lowpass_filter();
 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_{};
  ros::Subscriber sub_forcetorque_sensor_real;
  ros::Publisher pub_data;
  Eigen:: Matrix<double,6,1> err_force_int, err_force; // error in ft

  double tau_ = 1.0/(2*3.14*9.0);
  double filt_old_ = 0.0;
  double alphaTau = 1.0/(2*3.14*9.0);
  Eigen:: Matrix<double,7,1> tauFilt;
  double filt_ = 0.0;
  double delta_time = 0.002;
  double f_cur_buffer_ = 0.0;
};

}  // namespace franka_more_controllers
