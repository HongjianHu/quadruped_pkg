#include "quadruped_controller/FSM/StateTrotting.h"

#include "quadruped_controller/control/CtrlComponent.h"
#include "quadruped_controller/gait/WaveGenerator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace quadruped_controller
{

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::TROTTING, "TROTTING", ctrl_interfaces), ctrl_component_(ctrl_component)
{
}

void StateTrotting::enter()
{
    initialized_ = false;
    if (!ctrl_component_.robot_model_ || !ctrl_component_.wave_generator_)
    {
        return;
    }

    ctrl_component_.wave_generator_->status_ = WaveStatus::WAVE_ALL;
    const std::vector<SE3> init_foot_poses = ctrl_component_.robot_model_->getFeet2BPositions();
    for (int leg = 0; leg < 4; ++leg)
    {
        init_feet_body_.col(leg) = init_foot_poses[leg].translation();
        init_feet_body_(0, leg) += kBodyXCompensation;
        init_feet_body_(2, leg) -= kSupportExtension;
    }
    initialized_ = true;

    for (std::size_t i = 0; i < ctrl_interfaces_.joint_kp_command_interface_.size(); ++i)
    {
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.0);
    }
    for (std::size_t i = 0; i < ctrl_interfaces_.joint_kd_command_interface_.size(); ++i)
    {
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(0.0);
    }
    for (std::size_t i = 0; i < ctrl_interfaces_.joint_velocity_command_interface_.size(); ++i)
    {
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
    }

    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateTrotting::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!initialized_ || !ctrl_component_.robot_model_ || !ctrl_component_.wave_generator_)
    {
        return;
    }

    Vec34 feet_pos_body = init_feet_body_;
    Vec34 feet_vel_body = Vec34::Zero();
    std::vector<SE3> foot_poses(4, SE3::Identity());

    for (int leg = 0; leg < 4; ++leg)
    {
        if (ctrl_component_.wave_generator_->contact_(leg) == 0)
        {
            const double phase = ctrl_component_.wave_generator_->phase_(leg);
            const double phase_pi = 2.0 * M_PI * phase;
            feet_pos_body(2, leg) += kGaitHeight * (1.0 - std::cos(phase_pi)) * 0.5;
            feet_vel_body(2, leg) =
                kGaitHeight * M_PI * std::sin(phase_pi) / ctrl_component_.wave_generator_->getTSwing();
        }
        foot_poses[leg].translation() = feet_pos_body.col(leg);
    }

    const Vec12 target_joint_pos = ctrl_component_.robot_model_->getQ(feet_pos_body);
    const Vec12 target_joint_vel = ctrl_component_.robot_model_->getQd(foot_poses, feet_vel_body);

    const std::size_t joint_count = std::min<std::size_t>(12, ctrl_interfaces_.joint_torque_command_interface_.size());
    for (std::size_t i = 0; i < joint_count; ++i)
    {
        const int leg = static_cast<int>(i / 3);
        const int joint = static_cast<int>(i % 3);
        const double q = ctrl_component_.robot_model_->current_joint_pos_[leg][joint];
        const double qd = ctrl_component_.robot_model_->current_joint_vel_[leg][joint];
        double torque = kJointKp * (target_joint_pos[static_cast<int>(i)] - q) +
                        kJointKd * (target_joint_vel[static_cast<int>(i)] - qd);

        if (!std::isfinite(torque))
        {
            torque = 0.0;
        }
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque);
    }
}

void StateTrotting::exit()
{
    if (ctrl_component_.wave_generator_)
    {
        ctrl_component_.wave_generator_->status_ = WaveStatus::STANCE_ALL;
    }

    for (auto &cmd : ctrl_interfaces_.joint_torque_command_interface_)
    {
        cmd.get().set_value(0.0);
    }
}

FSMStateName StateTrotting::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDSTAND;
    default:
        return FSMStateName::TROTTING;
    }
}

} // namespace quadruped_controller
