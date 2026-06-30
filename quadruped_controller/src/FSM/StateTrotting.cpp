#include "quadruped_controller/FSM/StateTrotting.h"

#include "quadruped_controller/common/mathTools.h"
#include "quadruped_controller/control/CtrlComponent.h"
#include "quadruped_controller/gait/WaveGenerator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace quadruped_controller
{

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::TROTTING, "TROTTING", ctrl_interfaces), ctrl_component_(ctrl_component),
      gait_generator_(ctrl_component)
{
}

double StateTrotting::smoothStep(const double x)
{
    const double a = std::clamp(x, 0.0, 1.0);
    return a * a * (3.0 - 2.0 * a);
}

double StateTrotting::smoothStepDerivative(const double x)
{
    const double a = std::clamp(x, 0.0, 1.0);
    return 6.0 * a * (1.0 - a);
}

double StateTrotting::clampJointTorque(const double torque, const int joint)
{
    const double limit = joint == 2 ? kCalfTorqueLimit : (joint == 1 ? kThighTorqueLimit : kHipTorqueLimit);
    return std::clamp(torque, -limit, limit);
}

double StateTrotting::clampValue(const double value, const double limit)
{
    return std::clamp(value, -limit, limit);
}

void StateTrotting::enter()
{
    if (!ctrl_component_.robot_model_ || !ctrl_component_.estimator_ || !ctrl_component_.wave_generator_ ||
        !ctrl_component_.balance_ctrl_)
    {
        return;
    }

    elapsed_time_ = 0.0;
    next_balance_debug_time_ = 0.0;
    gait_started_ = false;
    ctrl_component_.wave_generator_->restart(WaveStatus::STANCE_ALL);
    entry_joint_pos_.setZero();
    stand_feedforward_torque_.assign(12, 0.0);
    desired_base_pos_ = ctrl_component_.estimator_->getPosition();
    desired_yaw_ = ctrl_component_.estimator_->getYaw();

    const std::vector<SE3> entry_foot_poses = ctrl_component_.robot_model_->getFeet2BPositions();
    for (int leg = 0; leg < 4; ++leg)
    {
        entry_feet_body_.col(leg) = entry_foot_poses[leg].translation();
        for (int joint = 0; joint < 3; ++joint)
        {
            entry_joint_pos_[leg * 3 + joint] = ctrl_component_.robot_model_->current_joint_pos_[leg][joint];
        }
    }

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
    for (std::size_t i = 0; i < stand_feedforward_torque_.size() && i < ctrl_interfaces_.joint_effort_state_interface_.size();
         ++i)
    {
        const double torque = ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value();
        stand_feedforward_torque_[i] = std::isfinite(torque) ? torque : 0.0;
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateTrotting::run(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
    if (!ctrl_component_.robot_model_ || !ctrl_component_.estimator_ || !ctrl_component_.wave_generator_ ||
        !ctrl_component_.balance_ctrl_)
    {
        return;
    }

    const double dt = period.seconds() > 0.0 ? period.seconds() : 1.0 / ctrl_interfaces_.frequency_;
    elapsed_time_ += dt;

    Vec34 feet_pos_body = entry_feet_body_;
    Vec34 feet_vel_body = Vec34::Zero();
    std::vector<SE3> foot_poses(4, SE3::Identity());
    Vec12 target_joint_pos = entry_joint_pos_;
    Vec12 target_joint_vel = Vec12::Zero();
    VecInt4 balance_contact = VecInt4::Ones();

    if (elapsed_time_ >= kEntryStanceDuration)
    {
        if (!gait_started_)
        {
            ctrl_component_.wave_generator_->restart(WaveStatus::WAVE_ALL);
            gait_generator_.restart();
            gait_generator_.setGait(Vec2::Zero(), 0.0, kGaitHeight);
            RCLCPP_INFO(rclcpp::get_logger("StateTrotting"),
                        "Trot phase reset: contact=[%d %d %d %d], phase=[%.3f %.3f %.3f %.3f]",
                        ctrl_component_.wave_generator_->contact_[0], ctrl_component_.wave_generator_->contact_[1],
                        ctrl_component_.wave_generator_->contact_[2], ctrl_component_.wave_generator_->contact_[3],
                        ctrl_component_.wave_generator_->phase_[0], ctrl_component_.wave_generator_->phase_[1],
                        ctrl_component_.wave_generator_->phase_[2], ctrl_component_.wave_generator_->phase_[3]);
            gait_started_ = true;
        }
        else
        {
            Vec34 feet_pos_world = Vec34::Zero();
            Vec34 feet_vel_world = Vec34::Zero();
            gait_generator_.generate(feet_pos_world, feet_vel_world);

            const Vec3 base_pos_world = ctrl_component_.estimator_->getPosition();
            const Vec3 base_vel_world = ctrl_component_.estimator_->getVelocity();
            const RotMat rotation_world_to_body = ctrl_component_.estimator_->getRotation().transpose();
            const Vec3 gyro_body = ctrl_component_.estimator_->getGyro();

            Vec34 gait_feet_pos_body = Vec34::Zero();
            Vec34 gait_feet_vel_body = Vec34::Zero();
            for (int leg = 0; leg < 4; ++leg)
            {
                gait_feet_pos_body.col(leg) = rotation_world_to_body * (feet_pos_world.col(leg) - base_pos_world);
                gait_feet_vel_body.col(leg) =
                    rotation_world_to_body * (feet_vel_world.col(leg) - base_vel_world) -
                    gyro_body.cross(gait_feet_pos_body.col(leg));
            }

            const double ramp_phase = (elapsed_time_ - kEntryStanceDuration) / kEntryRampDuration;
            const double alpha = smoothStep(ramp_phase);
            const double alpha_dot = smoothStepDerivative(ramp_phase) / kEntryRampDuration;
            feet_pos_body = (1.0 - alpha) * entry_feet_body_ + alpha * gait_feet_pos_body;
            feet_vel_body = alpha * gait_feet_vel_body + alpha_dot * (gait_feet_pos_body - entry_feet_body_);
            if (ramp_phase >= 1.0)
            {
                balance_contact = ctrl_component_.wave_generator_->contact_;
            }
        }
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        foot_poses[leg].translation() = feet_pos_body.col(leg);
    }

    if (elapsed_time_ >= kEntryStanceDuration && gait_started_)
    {
        target_joint_pos = ctrl_component_.robot_model_->getQ(feet_pos_body);
        target_joint_vel = ctrl_component_.robot_model_->getQd(foot_poses, feet_vel_body);
    }

    const Vec3 base_pos = ctrl_component_.estimator_->getPosition();
    const Vec3 base_vel = ctrl_component_.estimator_->getVelocity();
    const RotMat rotation = ctrl_component_.estimator_->getRotation();
    const Vec3 rpy = rotMatToRPY(rotation);
    const Vec3 gyro_world = ctrl_component_.estimator_->getGyroGlobal();

    Vec3 ddPcd = Vec3::Zero();
    ddPcd.x() = kBaseKpXY * (desired_base_pos_.x() - base_pos.x()) - kBaseKdXY * base_vel.x();
    ddPcd.y() = kBaseKpXY * (desired_base_pos_.y() - base_pos.y()) - kBaseKdXY * base_vel.y();
    ddPcd.z() = kBaseKpZ * (desired_base_pos_.z() - base_pos.z()) - kBaseKdZ * base_vel.z();
    ddPcd.x() = clampValue(ddPcd.x(), 1.5);
    ddPcd.y() = clampValue(ddPcd.y(), 1.5);
    ddPcd.z() = clampValue(ddPcd.z(), 5.0);

    Vec3 dWbd = Vec3::Zero();
    dWbd.x() = -kBaseKpRP * rpy.x() - kBaseKdRP * gyro_world.x();
    dWbd.y() = -kBaseKpRP * rpy.y() - kBaseKdRP * gyro_world.y();
    dWbd.z() = -kBaseKpYaw * (rpy.z() - desired_yaw_) - kBaseKdYaw * gyro_world.z();
    dWbd.x() = clampValue(dWbd.x(), 20.0);
    dWbd.y() = clampValue(dWbd.y(), 20.0);
    dWbd.z() = clampValue(dWbd.z(), 8.0);

    const Vec34 feet_pos_base_world = rotation * feet_pos_body;
    const Vec34 balance_force_world =
        ctrl_component_.balance_ctrl_->calF(ddPcd, dWbd, rotation, feet_pos_base_world, balance_contact);
    Vec12 balance_torque = Vec12::Zero();
    const double balance_blend = smoothStep(elapsed_time_ / kSupportBlendDuration);
    if (elapsed_time_ <= kEntryStanceDuration + kEntryRampDuration + 1.0 &&
        elapsed_time_ >= next_balance_debug_time_)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateTrotting"),
                    "Balance debug t=%.3f wave=[%d %d %d %d] balance=[%d %d %d %d] Fz=[%.2f %.2f %.2f %.2f] "
                    "base_z=%.3f roll=%.3f pitch=%.3f",
                    elapsed_time_, ctrl_component_.wave_generator_->contact_[0],
                    ctrl_component_.wave_generator_->contact_[1], ctrl_component_.wave_generator_->contact_[2],
                    ctrl_component_.wave_generator_->contact_[3], balance_contact[0], balance_contact[1],
                    balance_contact[2], balance_contact[3], balance_force_world(2, 0), balance_force_world(2, 1),
                    balance_force_world(2, 2), balance_force_world(2, 3), base_pos.z(), rpy.x(), rpy.y());
        next_balance_debug_time_ += 0.1;
    }
    for (int leg = 0; leg < 4; ++leg)
    {
        if (balance_contact[leg] == 0)
        {
            continue;
        }
        const Vec3 foot_force_body = rotation.transpose() * balance_force_world.col(leg);
        const Eigen::VectorXd leg_torque = ctrl_component_.robot_model_->getTorque(foot_force_body, leg);
        for (int joint = 0; joint < 3 && joint < leg_torque.size(); ++joint)
        {
            balance_torque[leg * 3 + joint] = leg_torque[joint];
        }
    }

    const std::size_t joint_count = std::min<std::size_t>(12, ctrl_interfaces_.joint_torque_command_interface_.size());
    for (std::size_t i = 0; i < joint_count; ++i)
    {
        const int leg = static_cast<int>(i / 3);
        const int joint = static_cast<int>(i % 3);
        const double q = ctrl_component_.robot_model_->current_joint_pos_[leg][joint];
        const double qd = ctrl_component_.robot_model_->current_joint_vel_[leg][joint];
        const double feedforward =
            i < stand_feedforward_torque_.size() ? stand_feedforward_torque_[i] : 0.0;
        const double support_torque = (1.0 - balance_blend) * feedforward + balance_blend * balance_torque[i];
        double torque = support_torque + kJointKp * (target_joint_pos[static_cast<int>(i)] - q) +
                        kJointKd * (target_joint_vel[static_cast<int>(i)] - qd);

        if (!std::isfinite(torque))
        {
            torque = 0.0;
        }
        torque = clampJointTorque(torque, joint);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque);
    }
}

void StateTrotting::exit()
{
    if (ctrl_component_.wave_generator_)
    {
        ctrl_component_.wave_generator_->restart(WaveStatus::STANCE_ALL);
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
