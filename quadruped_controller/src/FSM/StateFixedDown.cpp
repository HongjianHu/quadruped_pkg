#include "quadruped_controller/FSM/StateFixedDown.h"

namespace quadruped_controller {

StateFixedDown::StateFixedDown(CtrlInterfaces &ctrl_interfaces,
                               const std::vector<double> &down_pos, double kp,
                               double kd)
    : FSMState(FSMStateName::FIXEDDOWN, "FIXEDDOWN", ctrl_interfaces),
      target_pos_(down_pos), kp_(kp), kd_(kd), loop_count_(0) {}

void StateFixedDown::enter() { loop_count_ = 0; }

void StateFixedDown::run(const rclcpp::Time & /*time*/,
                         const rclcpp::Duration & /*period*/) {
  loop_count_++;

  // PD 控制：τ = kp*(q_des - q) + kd*(0 - q̇)
  for (size_t i = 0;
       i < ctrl_interfaces_.joint_torque_command_interface_.size(); ++i) {
    double pos =
        ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    double vel =
        ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();

    double torque = kp_ * (target_pos_[i] - pos) + kd_ * (0.0 - vel);

    ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque);
  }
}

void StateFixedDown::exit() {}

FSMStateName StateFixedDown::checkChange() {
  // 等机器人趴稳后才允许切换
  if (loop_count_ > settle_time_)
    return FSMStateName::PASSIVE;
  return FSMStateName::INVALID;
}

} // namespace quadruped_controller