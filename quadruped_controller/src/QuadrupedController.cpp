#include "quadruped_controller/QuadrupedController.h"
#include "quadruped_controller/gait/WaveGenerator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

namespace quadruped_controller {
using config_type = controller_interface::interface_configuration_type;

controller_interface::InterfaceConfiguration
QuadrupedController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL,
                                                       {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto &joint_name : joint_names_) {
    for (const auto &interface_type : command_interface_types_) {
      if (!command_prefix_.empty()) {
        conf.names.push_back(command_prefix_ + "/" + joint_name + "/" +
                             interface_type);
      } else {
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration
QuadrupedController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL,
                                                       {}};
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto &joint_name : joint_names_) {
    for (const auto &interface_type : state_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  for (const auto &interface_type : imu_interface_types_) {
    conf.names.push_back(imu_name_ + "/" + interface_type);
  }

  return conf;
}

controller_interface::return_type
QuadrupedController::update(const rclcpp::Time &time,
                            const rclcpp::Duration &period) {
  if (ctrl_component_.robot_model_ == nullptr) {
    return controller_interface::return_type::OK;
  }
  ctrl_component_.robot_model_->update();
  ctrl_component_.wave_generator_->update();
  ctrl_component_.estimator_->update();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn QuadrupedController::on_init() {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace quadruped_controller