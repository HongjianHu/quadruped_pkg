#include "quadruped_controller/FSM/StateMPCTrotting.h"

#include "quadruped_controller/common/mathTools.h"
#include "quadruped_controller/control/CtrlComponent.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace quadruped_controller
{

StateMPCTrotting::StateMPCTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::MPC_TROTTING, "MPC_TROTTING", ctrl_interfaces), ctrl_component_(ctrl_component),
      gait_(1.0 / kGaitPeriod, kGaitDuty), com_trajectory_(Vec3::Zero()), mpc_(kMpcHorizonSteps)
{
    gait_.setSwingHeight(kSwingHeight);
}

void StateMPCTrotting::enter()
{
    elapsed_time_ = 0.0;
    next_debug_time_ = 0.0;
    next_warning_time_ = 0.0;
    next_mpc_update_time_ = 0.0;
    has_mpc_solution_ = false;
    last_contact_forces_world_.setZero();
    leg_controller_ = LegController();

    ctrl_component_.gait_contact_.setOnes();
    ctrl_component_.gait_phase_.setConstant(0.5);

    RawBaseState base_state;
    const bool has_raw_base_state = readRawBaseState(base_state);
    if (has_raw_base_state)
    {
        desired_base_pos_world_ = base_state.pos_world;
        desired_base_pos_world_.z() = kDefaultBaseHeight;
        com_trajectory_.resetDesiredPosition(desired_base_pos_world_);
        syncPinModelWithRawState(base_state);
    }

    captureHoldPosition();
    captureNominalFootOffsets();
    commandTorqueControlDefaults();
    ctrl_interfaces_.control_inputs_.command = 0;

    RCLCPP_INFO(rclcpp::get_logger("StateMPCTrotting"),
                "Entered MPC_TROTTING; running ComTrajectory + CentroidalMPC + LegController");
}

void StateMPCTrotting::run(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
    if (!ctrl_component_.robot_model_)
    {
        commandZero();
        return;
    }

    const double dt = period.seconds() > 0.0 ? period.seconds() : 1.0 / ctrl_interfaces_.frequency_;
    elapsed_time_ += dt;

    RawBaseState base_state;
    if (!readRawBaseState(base_state) || !syncPinModelWithRawState(base_state))
    {
        commandZero();
        return;
    }

    if (elapsed_time_ < kEntryHoldDuration)
    {
        commandHoldPosition();
        if (elapsed_time_ >= next_debug_time_)
        {
            RCLCPP_INFO(rclcpp::get_logger("StateMPCTrotting"), "MPC entry hold t=%.3f raw_base=[%.3f %.3f %.3f]",
                        elapsed_time_, base_state.pos_world.x(), base_state.pos_world.y(), base_state.pos_world.z());
            next_debug_time_ += 0.5;
        }
        return;
    }

    auto &pin_model = ctrl_component_.robot_model_->pinModel();
    const RotMat yaw_rotation_body_to_world = rotz(base_state.rpy_world.z());

    updateDesiredCommand(dt, yaw_rotation_body_to_world);

    const double desired_x_vel_body =
        std::clamp(static_cast<double>(ctrl_interfaces_.control_inputs_.lx), -1.0, 1.0) * kMaxForwardVelocity;
    const double desired_y_vel_body =
        std::clamp(static_cast<double>(ctrl_interfaces_.control_inputs_.ly), -1.0, 1.0) * kMaxLateralVelocity;
    const double desired_yaw_rate_body =
        std::clamp(static_cast<double>(ctrl_interfaces_.control_inputs_.rx), -1.0, 1.0) * kMaxYawRate;

    Vec3 desired_velocity_body = Vec3::Zero();
    desired_velocity_body << desired_x_vel_body, desired_y_vel_body, 0.0;
    const Vec3 desired_velocity_world = yaw_rotation_body_to_world * desired_velocity_body;

    const double active_gait_time = gaitTime();
    const VecInt4 current_mask = gait_.computeCurrentMask(active_gait_time);

    updateSharedGaitState(active_gait_time, current_mask);
    const Vec34 touchdown_positions_world = buildTouchdownPositionsWorld(
        yaw_rotation_body_to_world, desired_velocity_world, desired_yaw_rate_body, base_state);

    if (!has_mpc_solution_ || elapsed_time_ >= next_mpc_update_time_)
    {
        ComTrajectory::Input input;
        input.initial_pos_world = base_state.pos_world;
        input.initial_rpy_world = base_state.rpy_world;
        input.yaw_rotation_body_to_world = yaw_rotation_body_to_world;
        input.initial_foot_levers_world = buildCurrentFootLeversWorld(base_state);
        input.hip_offsets_body = nominal_foot_offsets_body_;
        input.desired_x_vel_body = desired_x_vel_body;
        input.desired_y_vel_body = desired_y_vel_body;
        input.desired_z_pos_world = desired_base_pos_world_.z();
        input.desired_yaw_rate_body = desired_yaw_rate_body;
        input.mass = ctrl_component_.robot_model_->mass_;
        input.inertia_com_world = bodyInertiaWorld(base_state.rotation_body_to_world);
        input.horizon_time = kMpcHorizonTime;
        input.time_now = active_gait_time;
        input.time_step = kMpcTimeStep;

        Vec34 raw_contact_forces_world = Vec34::Zero();
        try
        {
            com_trajectory_.resetDesiredPosition(desired_base_pos_world_);
            com_trajectory_.generateReference(input, gait_);
            raw_contact_forces_world = mpc_.solveFirstForce(com_trajectory_, buildInitialMpcState(base_state));
        }
        catch (const std::exception &e)
        {
            if (elapsed_time_ >= next_warning_time_)
            {
                RCLCPP_WARN(rclcpp::get_logger("StateMPCTrotting"), "MPC solve failed, using fallback forces: %s",
                            e.what());
                next_warning_time_ = elapsed_time_ + 1.0;
            }
            raw_contact_forces_world = buildFallbackContactForces(current_mask);
        }
        last_contact_forces_world_ = sanitizeContactForces(raw_contact_forces_world, current_mask);
        has_mpc_solution_ = true;
        next_mpc_update_time_ = elapsed_time_ + kMpcUpdatePeriod;
    }
    last_contact_forces_world_ = enforceCurrentContactMask(last_contact_forces_world_, current_mask);

    leg_controller_.setTouchdownPositionsWorld(touchdown_positions_world);

    Vec12 joint_torques = Vec12::Zero();
    for (int leg = 0; leg < 4; ++leg)
    {
        const LegOutput leg_output = leg_controller_.computeLegTorque(
            leg, pin_model, gait_, last_contact_forces_world_.col(leg), active_gait_time);

        for (int joint = 0; joint < 3; ++joint)
        {
            joint_torques[3 * leg + joint] = clampJointTorque(leg_output.tau[joint], joint);
        }
    }

    commandTorqueControlDefaults();
    writeJointTorques(joint_torques);

    if (elapsed_time_ >= next_debug_time_)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateMPCTrotting"),
                    "MPC t=%.3f mask=[%d %d %d %d] raw_base=[%.3f %.3f %.3f] raw_v=[%.3f %.3f %.3f] Fz=[%.1f %.1f %.1f "
                    "%.1f] tau_norm=%.2f",
                    elapsed_time_, current_mask[0], current_mask[1], current_mask[2], current_mask[3],
                    base_state.pos_world.x(), base_state.pos_world.y(), base_state.pos_world.z(),
                    base_state.linear_vel_world.x(), base_state.linear_vel_world.y(), base_state.linear_vel_world.z(),
                    last_contact_forces_world_(2, 0), last_contact_forces_world_(2, 1),
                    last_contact_forces_world_(2, 2), last_contact_forces_world_(2, 3), joint_torques.norm());
        next_debug_time_ += 0.5;
    }
}

void StateMPCTrotting::exit()
{
    // 进入FixedStand或FreeStand后，Estimator不会继续按照交替小跑接触进行门控
    ctrl_component_.gait_contact_.setOnes();
    ctrl_component_.gait_phase_.setConstant(0.5);

    commandZero();
}

FSMStateName StateMPCTrotting::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDSTAND;
    case 4:
        return FSMStateName::FREESTAND;
    default:
        return FSMStateName::MPC_TROTTING;
    }
}

void StateMPCTrotting::captureHoldPosition()
{
    hold_joint_pos_.setZero();
    if (!ctrl_component_.robot_model_)
    {
        return;
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            hold_joint_pos_[leg * 3 + joint] = ctrl_component_.robot_model_->current_joint_pos_[leg][joint];
        }
    }
}

void StateMPCTrotting::captureNominalFootOffsets()
{
    nominal_foot_offsets_body_.setZero();
    if (!ctrl_component_.robot_model_)
    {
        return;
    }

    const auto &pin_model = ctrl_component_.robot_model_->pinModel();
    for (int leg = 0; leg < 4; ++leg)
    {
        nominal_foot_offsets_body_.col(leg) = pin_model.footPoseBody(leg).translation();
        nominal_foot_offsets_body_(2, leg) = 0.0;
    }
}

bool StateMPCTrotting::readRawBaseState(RawBaseState &state) const
{
    if (ctrl_interfaces_.odometer_state_interface_.size() < 6 || ctrl_interfaces_.imu_state_interface_.size() < 7)
    {
        return false;
    }

    state.pos_world << ctrl_interfaces_.odometer_state_interface_[0].get().get_value(),
        ctrl_interfaces_.odometer_state_interface_[1].get().get_value(),
        ctrl_interfaces_.odometer_state_interface_[2].get().get_value();

    state.linear_vel_world << ctrl_interfaces_.odometer_state_interface_[3].get().get_value(),
        ctrl_interfaces_.odometer_state_interface_[4].get().get_value(),
        ctrl_interfaces_.odometer_state_interface_[5].get().get_value();

    Quat quat_wxyz;
    quat_wxyz << ctrl_interfaces_.imu_state_interface_[0].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[1].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[2].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    state.rotation_body_to_world = quatToRotMat(quat_wxyz);
    state.rpy_world = rotMatToRPY(state.rotation_body_to_world);

    state.angular_vel_body << ctrl_interfaces_.imu_state_interface_[4].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[5].get().get_value(),
        ctrl_interfaces_.imu_state_interface_[6].get().get_value();
    state.angular_vel_world = state.rotation_body_to_world * state.angular_vel_body;

    for (int axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(state.pos_world[axis]) || !std::isfinite(state.linear_vel_world[axis]) ||
            !std::isfinite(state.angular_vel_body[axis]))
        {
            return false;
        }
    }
    return true;
}

bool StateMPCTrotting::syncPinModelWithRawState(const RawBaseState &state)
{
    if (!ctrl_component_.robot_model_)
    {
        return false;
    }

    ctrl_component_.robot_model_->updatePinModelWithBase(state.pos_world, state.rotation_body_to_world,
                                                         state.linear_vel_world, state.angular_vel_body);
    return true;
}

double StateMPCTrotting::gaitTime() const
{
    return std::max(0.0, elapsed_time_ - kEntryHoldDuration);
}
// Estimator需要阶段内部相位，windowFunc是相对于摆动腿的摆动进度改变置信度的，而不是一个gait周期，所以需要归一化
void StateMPCTrotting::updateSharedGaitState(const double gait_time, const VecInt4 &contact)
{
    const Vec4 cycle_phase = gait_.computePhase(gait_time);

    ctrl_component_.gait_contact_ = contact;

    for (int leg = 0; leg < 4; ++leg)
    {
        if (contact[leg] == 1)
        {
            ctrl_component_.gait_phase_[leg] = cycle_phase[leg] / gait_.duty();
        }
        else
        {
            ctrl_component_.gait_phase_[leg] = (cycle_phase[leg] - gait_.duty()) / (1 - gait_.duty());
        }
    }
}

void StateMPCTrotting::updateDesiredCommand(const double dt, const RotMat &yaw_rotation_body_to_world)
{
    const double desired_x_vel_body =
        std::clamp(static_cast<double>(ctrl_interfaces_.control_inputs_.lx), -1.0, 1.0) * kMaxForwardVelocity;
    const double desired_y_vel_body =
        std::clamp(static_cast<double>(ctrl_interfaces_.control_inputs_.ly), -1.0, 1.0) * kMaxLateralVelocity;
    const double desired_z_vel_world =
        std::clamp(static_cast<double>(ctrl_interfaces_.control_inputs_.ry), -1.0, 1.0) * kMaxHeightRate;

    Vec3 desired_velocity_body = Vec3::Zero();
    desired_velocity_body << desired_x_vel_body, desired_y_vel_body, 0.0;
    const Vec3 desired_velocity_world = yaw_rotation_body_to_world * desired_velocity_body;

    desired_base_pos_world_.x() += desired_velocity_world.x() * dt;
    desired_base_pos_world_.y() += desired_velocity_world.y() * dt;
    desired_base_pos_world_.z() =
        std::clamp(desired_base_pos_world_.z() + desired_z_vel_world * dt, kMinBaseHeight, kMaxBaseHeight);
}

Vec34 StateMPCTrotting::buildCurrentFootLeversWorld(const RawBaseState &base_state) const
{
    Vec34 foot_levers_world = Vec34::Zero();
    if (!ctrl_component_.robot_model_)
    {
        return foot_levers_world;
    }

    const auto &pin_model = ctrl_component_.robot_model_->pinModel();
    for (int leg = 0; leg < 4; ++leg)
    {
        foot_levers_world.col(leg) = pin_model.footPositionWorld(leg) - base_state.pos_world;
    }
    return foot_levers_world;
}

Vec34 StateMPCTrotting::buildTouchdownPositionsWorld(const RotMat &yaw_rotation_body_to_world,
                                                     const Vec3 &desired_velocity_world, const double desired_yaw_rate,
                                                     const RawBaseState &base_state) const
{
    Gait::TouchdownInput touchdown_input;
    touchdown_input.base_pos_world = base_state.pos_world;
    touchdown_input.com_pos_world = touchdown_input.base_pos_world;
    touchdown_input.com_vel_world = base_state.linear_vel_world;
    touchdown_input.yaw_rotation_body_to_world = yaw_rotation_body_to_world;
    touchdown_input.desired_velocity_world = desired_velocity_world;
    touchdown_input.desired_position_world = desired_base_pos_world_;
    touchdown_input.yaw_rate_des_world = desired_yaw_rate;

    return gait_.computeTouchdownWorlds(touchdown_input, nominal_foot_offsets_body_);
}

Vec12 StateMPCTrotting::buildInitialMpcState(const RawBaseState &base_state) const
{
    Vec12 initial_state = Vec12::Zero();
    initial_state.segment<3>(0) = base_state.pos_world;
    initial_state.segment<3>(3) = base_state.rpy_world;
    initial_state.segment<3>(6) = base_state.linear_vel_world;
    initial_state.segment<3>(9) = base_state.angular_vel_world;
    return initial_state;
}

Vec34 StateMPCTrotting::buildFallbackContactForces(const VecInt4 &mask) const
{
    Vec34 forces = Vec34::Zero();
    if (!ctrl_component_.robot_model_)
    {
        return forces;
    }

    const int contact_count = std::max(1, mask.sum());
    const double fz = ctrl_component_.robot_model_->mass_ * 9.81 / static_cast<double>(contact_count);

    for (int leg = 0; leg < 4; ++leg)
    {
        if (mask[leg] == 1)
        {
            forces(2, leg) = fz;
        }
    }

    return forces;
}

Vec34 StateMPCTrotting::sanitizeContactForces(const Vec34 &forces_world, const VecInt4 &mask) const
{
    Vec34 sanitized = forces_world;

    for (int leg = 0; leg < 4; ++leg)
    {
        if (mask[leg] == 0)
        {
            sanitized.col(leg).setZero();
            continue;
        }

        for (int axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(sanitized(axis, leg)))
            {
                sanitized(axis, leg) = 0.0;
            }
        }

        sanitized(2, leg) =
            std::clamp(sanitized(2, leg), CentroidalMPC::kMinNormalForce, CentroidalMPC::kMaxNormalForce);
        const double tangential_limit = CentroidalMPC::kFrictionCoefficient * sanitized(2, leg);
        const double tangential_l1 = std::abs(sanitized(0, leg)) + std::abs(sanitized(1, leg));
        if (tangential_l1 > tangential_limit && tangential_l1 > 1.0e-9)
        {
            const double scale = tangential_limit / tangential_l1;
            sanitized(0, leg) *= scale;
            sanitized(1, leg) *= scale;
        }
    }

    return sanitized;
}

Vec34 StateMPCTrotting::enforceCurrentContactMask(const Vec34 &forces_world, const VecInt4 &mask) const
{
    Vec34 forces = forces_world;
    for (int leg = 0; leg < 4; ++leg)
    {
        if (mask[leg] == 0)
        {
            forces.col(leg).setZero();
        }
    }

    return forces;
}

Mat3 StateMPCTrotting::bodyInertiaWorld(const RotMat &base_rotation_body_to_world)
{
    const Mat3 body_inertia = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
    return base_rotation_body_to_world * body_inertia * base_rotation_body_to_world.transpose();
}

double StateMPCTrotting::clampJointTorque(const double torque, const int joint)
{
    const double limit = joint == 2 ? kCalfTorqueLimit : (joint == 1 ? kThighTorqueLimit : kHipTorqueLimit);
    return std::clamp(std::isfinite(torque) ? torque : 0.0, -limit, limit);
}

void StateMPCTrotting::commandHoldPosition()
{
    const std::size_t joint_count =
        std::min<std::size_t>(12, ctrl_interfaces_.joint_position_command_interface_.size());

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(hold_joint_pos_[static_cast<int>(i)]);
    }

    for (std::size_t i = 0; i < std::min<std::size_t>(12, ctrl_interfaces_.joint_velocity_command_interface_.size());
         ++i)
    {
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
    }

    for (std::size_t i = 0; i < std::min<std::size_t>(12, ctrl_interfaces_.joint_torque_command_interface_.size()); ++i)
    {
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
    }

    for (std::size_t i = 0; i < std::min<std::size_t>(12, ctrl_interfaces_.joint_kp_command_interface_.size()); ++i)
    {
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kHoldKp);
    }

    for (std::size_t i = 0; i < std::min<std::size_t>(12, ctrl_interfaces_.joint_kd_command_interface_.size()); ++i)
    {
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kHoldKd);
    }
}

void StateMPCTrotting::commandTorqueControlDefaults()
{
    const std::size_t joint_count = std::min<std::size_t>(12, ctrl_interfaces_.joint_torque_command_interface_.size());

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        if (i < ctrl_interfaces_.joint_position_command_interface_.size())
        {
            const int leg = static_cast<int>(i / 3);
            const int joint = static_cast<int>(i % 3);
            const double q = ctrl_component_.robot_model_ ? ctrl_component_.robot_model_->current_joint_pos_[leg][joint]
                                                          : hold_joint_pos_[static_cast<int>(i)];
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q);
        }
        if (i < ctrl_interfaces_.joint_velocity_command_interface_.size())
        {
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        }
        if (i < ctrl_interfaces_.joint_kp_command_interface_.size())
        {
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.0);
        }
        if (i < ctrl_interfaces_.joint_kd_command_interface_.size())
        {
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(0.0);
        }
    }
}

void StateMPCTrotting::writeJointTorques(const Vec12 &joint_torques)
{
    const std::size_t joint_count = std::min<std::size_t>(12, ctrl_interfaces_.joint_torque_command_interface_.size());

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(joint_torques[static_cast<int>(i)]);
    }
}

void StateMPCTrotting::commandZero()
{
    for (auto &cmd : ctrl_interfaces_.joint_torque_command_interface_)
    {
        cmd.get().set_value(0.0);
    }
    for (auto &cmd : ctrl_interfaces_.joint_kp_command_interface_)
    {
        cmd.get().set_value(0.0);
    }
    for (auto &cmd : ctrl_interfaces_.joint_kd_command_interface_)
    {
        cmd.get().set_value(0.0);
    }
    for (auto &cmd : ctrl_interfaces_.joint_velocity_command_interface_)
    {
        cmd.get().set_value(0.0);
    }
}

} // namespace quadruped_controller
