#include "quadruped_controller/FSM/StateFixedStand.h"

#include <algorithm>
#include <cmath>

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
    elapsed_time_ = 0.0;
    init_pos_.assign(target_pos_.size(), 0.0);
    for (std::size_t i = 0; i < init_pos_.size() && i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
    {
        init_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
}

void StateFixedStand::run(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
    loop_count_++;
    const double dt = period.seconds() > 0.0 ? period.seconds() : 0.005;
    elapsed_time_ += dt;
    const double alpha = std::clamp(elapsed_time_ / kStandRampDuration, 0.0, 1.0);
    const double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);

    for (size_t i = 0; i < ctrl_interfaces_.joint_torque_command_interface_.size(); ++i)
    {
        double pos = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        double vel = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        const double target = init_pos_[i] + smooth_alpha * (target_pos_[i] - init_pos_[i]);
        double torque = kp_ * (target - pos) + kd_ * (0.0 - vel);
        if (!std::isfinite(torque))
        {
            torque = 0.0;
        }
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque);
    }
}

void StateFixedStand::exit()
{
}

FSMStateName StateFixedStand::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 4:
        return FSMStateName::FREESTAND;
    case 6:
        return FSMStateName::MPC_TROTTING;
    case 7:
        return FSMStateName::MPC_WBC_TROTTING;
    default:
        return FSMStateName::FIXEDSTAND;
    }
}

} // namespace quadruped_controller
