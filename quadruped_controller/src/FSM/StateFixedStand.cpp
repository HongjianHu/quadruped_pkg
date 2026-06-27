#include "quadruped_controller/FSM/StateFixedStand.h"

namespace quadruped_controller
{

StateFixedStand::StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &stand_pos, double kp,
                                 double kd)
    : FSMState(FSMStateName::FIXEDSTAND, "FIXEDSTAND", ctrl_interfaces), target_pos_(stand_pos), kp_(kp), kd_(kd),
      loop_count_(0)
{
}

void StateFixedStand::enter()
{
    loop_count_ = 0;
}

void StateFixedStand::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    loop_count_++;

    for (size_t i = 0; i < ctrl_interfaces_.joint_torque_command_interface_.size(); ++i)
    {
        double pos = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        double vel = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        double torque = kp_ * (target_pos_[i] - pos) + kd_ * (0.0 - vel);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque);
    }
}

void StateFixedStand::exit()
{
}

FSMStateName StateFixedStand::checkChange()
{
    if (loop_count_ > settle_time_)
        return FSMStateName::FREESTAND; // 站稳后 → 自由站立
    return FSMStateName::INVALID;
}

} // namespace quadruped_controller
