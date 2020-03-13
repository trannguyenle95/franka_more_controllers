/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 25/10/19
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */
 #include <franka_more_controllers/GravityCompensation.h>
 #include <pluginlib/class_list_macros.h>
 #include <math.h>
 #include <Eigen/LU>

bool GravityCompensation::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &handle) {
    const auto names = hw->getNames();
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto jname = std::string("panda_joint") + std::to_string(i + 1);
        if (std::find(names.begin(), names.end(), jname) == names.end()) {
            ROS_ERROR_STREAM("Joint not found: " << jname);
            ROS_ERROR_STREAM("Available joints: ");
            for (const auto &name : names) {
                ROS_ERROR_STREAM(name);
            }
            return false;
        }
        joints[i] = hw->getHandle(jname);
    }
    return Controller::init(hw, handle);
}

void GravityCompensation::starting(const ros::Time &time1) {
    ControllerBase::starting(time1);
}

void GravityCompensation::update(const ros::Time &time, const ros::Duration &period) {
   Eigen::VectorXd tau(7);
   tau << 0,0,0,0,0,0,0;
   Eigen::VectorXd torque_limits(7);
   torque_limits << 87, 87, 87, 87, 12, 12, 12; // torque limits for joints
   for (int i = 0; i < 7; ++i){
       // check torque limitations
       if (tau[i] > torque_limits[i])
           tau[i] = torque_limits[i];
       else if (tau[i] < -torque_limits[i])
           tau[i] = -torque_limits[i];
   }
   for (size_t i = 0; i < joints.size(); ++i) {
      joints[i].setCommand(tau[i]); //send torque command to control the robot
    }
}
void GravityCompensation::stopping(const ros::Time &time1) {
    ControllerBase::stopping(time1);
}

PLUGINLIB_EXPORT_CLASS(GravityCompensation, controller_interface::ControllerBase)
