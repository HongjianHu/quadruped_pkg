#include "quadruped_controller/FSM/StateMPCWBCTrotting.h"

#include "quadruped_controller/common/mathTools.h"
#include "quadruped_controller/control/CtrlComponent.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace quadruped_controller
{

StateMPCWBCTrotting::StateMPCWBCTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::MPC_WBC_TROTTING, "MPC_WBC_TROTTING", ctrl_interfaces), ctrl_component_(ctrl_component),
      gait_(1.0 / kGaitPeriod, kGaitDuty), com_trajectory_(Vec3::Zero()), mpc_(kMpcHorizonSteps)
{
    gait_.setSwingHeight(kSwingHeight);
}

void StateMPCWBCTrotting::enter()
{
    elapsed_time_ = 0.0;
    next_debug_time_ = 0.0;
    next_warning_time_ = 0.0;
    next_wbc_warning_time_ = 0.0;
    next_mpc_update_time_ = 0.0;
    has_mpc_solution_ = false;
    last_mpc_contact_.setZero();
    last_contact_forces_world_.setZero();
    last_wbc_output_ = WbcOutput();
    wbc_solve_count_ = 0;
    wbc_success_count_ = 0;
    wbc_failure_count_ = 0;
    last_wbc_solve_time_us_ = 0.0;
    accumulated_wbc_solve_time_us_ = 0.0;
    resetTorqueAuthority();
    wbc_fallback_event_count_ = 0;
    last_raw_foot_normal_forces_.setZero();
    last_wbc_contact_.setZero();
    last_desired_base_acceleration_tangent_.setZero();
    last_base_vertical_velocity_ = 0.0;
    filtered_actual_base_acceleration_z_ = 0.0;
    has_previous_base_vertical_velocity_ = false;
    last_commanded_torques_.setZero();
    fallback_leg_controller_ = LegController();

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
    if (has_raw_base_state && ctrl_component_.robot_model_)
    {
        Vec34 current_foot_positions_world = Vec34::Zero();
        auto &pin_model = ctrl_component_.robot_model_->pinModel();
        for (int leg = 0; leg < 4; ++leg)
        {
            current_foot_positions_world.col(leg) = pin_model.footPositionWorld(leg);
        }
        gait_.resetSwingState(current_foot_positions_world);
    }
    JointImpedanceReference entry_reference;
    entry_reference.position = hold_joint_pos_;
    commandHybridTorqueControl(entry_reference, VecInt4::Ones(), 0.0);
    ctrl_interfaces_.control_inputs_.command = 0;

    RCLCPP_INFO(rclcpp::get_logger("StateMPCWBCTrotting"),
                "Entered MPC_WBC_TROTTING; LegController is the initial authority, then WBC takes over through a "
                "guarded torque blend");
}

void StateMPCWBCTrotting::run(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
{
    if (!ctrl_component_.robot_model_)
    {
        resetTorqueAuthority();
        last_commanded_torques_.setZero();
        commandZero();
        return;
    }

    const double dt = period.seconds() > 0.0 ? period.seconds() : 1.0 / ctrl_interfaces_.frequency_;
    elapsed_time_ += dt;

    RawBaseState base_state;
    if (!readRawBaseState(base_state) || !syncPinModelWithRawState(base_state))
    {
        resetTorqueAuthority();
        last_commanded_torques_.setZero();
        commandZero();
        return;
    }

    if (has_previous_base_vertical_velocity_)
    {
        const double raw_acceleration_z =
            (base_state.linear_vel_world.z() - last_base_vertical_velocity_) / std::max(dt, 1.0e-6);
        const double filter_alpha =
            std::clamp(dt / (kActualAccelerationFilterTimeConstant + dt), 0.0, 1.0);
        filtered_actual_base_acceleration_z_ +=
            filter_alpha * (raw_acceleration_z - filtered_actual_base_acceleration_z_);
    }
    else
    {
        has_previous_base_vertical_velocity_ = true;
    }
    last_base_vertical_velocity_ = base_state.linear_vel_world.z();

    if (elapsed_time_ < kEntryHoldDuration)
    {
        commandHoldPosition();
        if (elapsed_time_ >= next_debug_time_)
        {
            RCLCPP_INFO(rclcpp::get_logger("StateMPCWBCTrotting"), "MPC entry hold t=%.3f raw_base=[%.3f %.3f %.3f]",
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
    last_wbc_contact_ = current_mask;
    last_raw_foot_normal_forces_ = readFootNormalForces();

    updateSharedGaitState(active_gait_time, current_mask);
    const Vec34 touchdown_positions_world = buildTouchdownPositionsWorld(
        yaw_rotation_body_to_world, desired_velocity_world, desired_yaw_rate_body, base_state);

    const bool mpc_contact_changed =
        has_mpc_solution_ && (current_mask.array() != last_mpc_contact_.array()).any();

    if (!has_mpc_solution_ || elapsed_time_ >= next_mpc_update_time_ || mpc_contact_changed)
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
                RCLCPP_WARN(rclcpp::get_logger("StateMPCWBCTrotting"), "MPC solve failed, using fallback forces: %s",
                            e.what());
                next_warning_time_ = elapsed_time_ + 1.0;
            }
            raw_contact_forces_world = buildFallbackContactForces(current_mask);
        }
        last_contact_forces_world_ = sanitizeContactForces(raw_contact_forces_world, current_mask);
        has_mpc_solution_ = true;
        last_mpc_contact_ = current_mask;
        next_mpc_update_time_ = elapsed_time_ + kMpcUpdatePeriod;
    }
    last_contact_forces_world_ = enforceCurrentContactMask(last_contact_forces_world_, current_mask);

    fallback_leg_controller_.setTouchdownPositionsWorld(touchdown_positions_world);

    const FootTrajectoryReference foot_reference =
        buildFootTrajectoryReference(active_gait_time, current_mask, touchdown_positions_world);
    const JointImpedanceReference joint_reference = buildJointImpedanceReference(base_state, foot_reference);

    const auto wbc_start_time = std::chrono::steady_clock::now();
    try
    {
        const WbcInput wbc_input =
            buildWbcInput(base_state, last_wbc_contact_, foot_reference.acceleration_world);

        WbcWeights runtime_weights;
        runtime_weights.base_acceleration = 2000.0;
        runtime_weights.swing_foot_acceleration = 200.0;
        runtime_weights.joint_acceleration = 0.05;
        runtime_weights.contact_force_tracking = 0.01;

        last_wbc_output_ = wbc_controller_.solve(wbc_input, runtime_weights);
    }
    catch (const std::exception &exception)
    {
        last_wbc_output_ = WbcOutput();
        last_wbc_output_.status = std::string("WBC input construction failed: ") + exception.what();
    }
    const auto wbc_end_time = std::chrono::steady_clock::now();
    last_wbc_solve_time_us_ = std::chrono::duration<double, std::micro>(wbc_end_time - wbc_start_time).count();
    accumulated_wbc_solve_time_us_ += last_wbc_solve_time_us_;
    ++wbc_solve_count_;

    if (last_wbc_output_.success)
    {
        ++wbc_success_count_;
    }
    else
    {
        ++wbc_failure_count_;
        if (elapsed_time_ >= next_wbc_warning_time_)
        {
            RCLCPP_WARN(rclcpp::get_logger("StateMPCWBCTrotting"),
                        "WBC solve failed; torque target is falling back to LegController: %s "
                        "mask=[%d %d %d %d] res_dyn=%.3e res_eq=%.3e viol=%.3e",
                        last_wbc_output_.status.c_str(), current_mask[0], current_mask[1], current_mask[2],
                        current_mask[3], last_wbc_output_.dynamics_residual_norm,
                        last_wbc_output_.equality_residual_norm, last_wbc_output_.max_inequality_violation);
            next_wbc_warning_time_ = elapsed_time_ + 1.0;
        }
    }

    Vec12 fallback_joint_torques = Vec12::Zero();
    for (int leg = 0; leg < 4; ++leg)
    {
        const LegOutput leg_output = fallback_leg_controller_.computeLegTorque(
            leg, pin_model, gait_, last_contact_forces_world_.col(leg), active_gait_time);

        for (int joint = 0; joint < 3; ++joint)
        {
            fallback_joint_torques[3 * leg + joint] = clampJointTorque(leg_output.tau[joint], joint);
        }
    }

    const bool base_state_safe = isBaseStateSafeForWbc(base_state);
    if (!base_state_safe && elapsed_time_ >= next_wbc_warning_time_)
    {
        RCLCPP_WARN(rclcpp::get_logger("StateMPCWBCTrotting"),
                    "WBC takeover blocked by base-state guard: z=%.3f roll=%.3f pitch=%.3f", base_state.pos_world.z(),
                    base_state.rpy_world.x(), base_state.rpy_world.y());
        next_wbc_warning_time_ = elapsed_time_ + 1.0;
    }

    const Vec12 commanded_joint_torques =
        selectCommandedTorques(fallback_joint_torques, last_wbc_output_, last_wbc_contact_, base_state_safe, dt);

    commandHybridTorqueControl(joint_reference, current_mask, wbc_blend_);
    writeJointTorques(commanded_joint_torques);

    if (elapsed_time_ >= next_debug_time_)
    {
        const double average_wbc_solve_time_us =
            wbc_solve_count_ > 0 ? accumulated_wbc_solve_time_us_ / static_cast<double>(wbc_solve_count_) : 0.0;
        const double wbc_torque_norm = last_wbc_output_.success ? last_wbc_output_.joint_torques.norm() : 0.0;
        const double torque_difference_norm =
            last_wbc_output_.success ? (last_wbc_output_.joint_torques - fallback_joint_torques).norm() : 0.0;
        const Vec3 base_linear_velocity_body =
            base_state.rotation_body_to_world.transpose() * base_state.linear_vel_world;
        Vec3 predicted_base_linear_acceleration_world = Vec3::Zero();
        Vec3 desired_base_linear_acceleration_world = Vec3::Zero();
        if (last_wbc_output_.success)
        {
            predicted_base_linear_acceleration_world =
                base_state.rotation_body_to_world * (last_wbc_output_.generalized_acceleration.head<3>() +
                                                     base_state.angular_vel_body.cross(base_linear_velocity_body));
        }
        desired_base_linear_acceleration_world =
            base_state.rotation_body_to_world *
            (last_desired_base_acceleration_tangent_.head<3>() +
             base_state.angular_vel_body.cross(base_linear_velocity_body));

        const double robot_mass = ctrl_component_.robot_model_->mass_;
        const double measured_force_acceleration_z =
            robot_mass > 0.0 ? last_raw_foot_normal_forces_.sum() / robot_mass - 9.81 : 0.0;
        const double wbc_force_acceleration_z =
            robot_mass > 0.0 ? last_wbc_output_.contact_forces_world.row(2).sum() / robot_mass - 9.81 : 0.0;

        double max_wbc_torque_utilization = 0.0;
        double max_commanded_torque_utilization = 0.0;
        for (int leg = 0; leg < 4; ++leg)
        {
            for (int joint = 0; joint < 3; ++joint)
            {
                const int index = 3 * leg + joint;
                const double limit = jointTorqueLimit(joint);
                max_wbc_torque_utilization =
                    std::max(max_wbc_torque_utilization, std::abs(last_wbc_output_.joint_torques[index]) / limit);
                max_commanded_torque_utilization =
                    std::max(max_commanded_torque_utilization, std::abs(commanded_joint_torques[index]) / limit);
            }
        }
        RCLCPP_INFO(
            rclcpp::get_logger("StateMPCWBCTrotting"),
            "MPC+WBC t=%.3f base=[z %.3f vz %.3f r %.3f p %.3f] contact=[%d %d %d %d] "
            "foot_force_raw=[%.1f %.1f %.1f %.1f] "
            "Fz_mpc=[%.1f %.1f %.1f %.1f] Fz_wbc=[%.1f %.1f %.1f %.1f] "
            "az=[des %.2f qp %.2f force_qp %.2f force_mj %.2f actual %.2f] "
            "wbc=%s ok/fail=%zu/%zu solve_us=%.1f avg_us=%.1f init=%zu "
            "authority=%s blend=%.3f qualify=[%zu cycles %zu switches] fallback_events=%zu "
            "res_dyn=%.2e res_eq=%.2e viol=%.2e tau_leg/wbc/cmd/diff=[%.2f %.2f %.2f %.2f] "
            "tau_util=[wbc %.2f cmd %.2f]",
            elapsed_time_, base_state.pos_world.z(), base_state.linear_vel_world.z(), base_state.rpy_world.x(),
            base_state.rpy_world.y(), current_mask[0], current_mask[1], current_mask[2], current_mask[3],
            last_raw_foot_normal_forces_[0],
            last_raw_foot_normal_forces_[1], last_raw_foot_normal_forces_[2], last_raw_foot_normal_forces_[3],
            last_contact_forces_world_(2, 0), last_contact_forces_world_(2, 1), last_contact_forces_world_(2, 2),
            last_contact_forces_world_(2, 3), last_wbc_output_.contact_forces_world(2, 0),
            last_wbc_output_.contact_forces_world(2, 1), last_wbc_output_.contact_forces_world(2, 2),
            last_wbc_output_.contact_forces_world(2, 3), desired_base_linear_acceleration_world.z(),
            predicted_base_linear_acceleration_world.z(), wbc_force_acceleration_z, measured_force_acceleration_z,
            filtered_actual_base_acceleration_z_,
            last_wbc_output_.success ? "solved" : last_wbc_output_.status.c_str(), wbc_success_count_,
            wbc_failure_count_, last_wbc_solve_time_us_, average_wbc_solve_time_us,
            wbc_controller_.solverInitializationCount(), torqueAuthorityName(), wbc_blend_, consecutive_wbc_successes_,
            qualified_contact_switches_, wbc_fallback_event_count_, last_wbc_output_.dynamics_residual_norm,
            last_wbc_output_.equality_residual_norm, last_wbc_output_.max_inequality_violation,
            fallback_joint_torques.norm(), wbc_torque_norm, commanded_joint_torques.norm(), torque_difference_norm,
            max_wbc_torque_utilization, max_commanded_torque_utilization);
        next_debug_time_ += 0.5;
    }
}

void StateMPCWBCTrotting::exit()
{
    // 进入FixedStand或FreeStand后，Estimator不会继续按照交替小跑接触进行门控
    ctrl_component_.gait_contact_.setOnes();
    ctrl_component_.gait_phase_.setConstant(0.5);

    resetTorqueAuthority();
    last_commanded_torques_.setZero();
    commandZero();
}

FSMStateName StateMPCWBCTrotting::checkChange()
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
        return FSMStateName::MPC_WBC_TROTTING;
    }
}

void StateMPCWBCTrotting::captureHoldPosition()
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

void StateMPCWBCTrotting::captureNominalFootOffsets()
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

bool StateMPCWBCTrotting::readRawBaseState(RawBaseState &state) const
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

bool StateMPCWBCTrotting::syncPinModelWithRawState(const RawBaseState &state)
{
    if (!ctrl_component_.robot_model_)
    {
        return false;
    }

    ctrl_component_.robot_model_->updatePinModelWithBase(state.pos_world, state.rotation_body_to_world,
                                                         state.linear_vel_world, state.angular_vel_body);
    return true;
}

double StateMPCWBCTrotting::gaitTime() const
{
    return std::max(0.0, elapsed_time_ - kEntryHoldDuration);
}
// Estimator需要阶段内部相位，windowFunc是相对于摆动腿的摆动进度改变置信度的，而不是一个gait周期，所以需要归一化
void StateMPCWBCTrotting::updateSharedGaitState(const double gait_time, const VecInt4 &contact)
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

Vec4 StateMPCWBCTrotting::readFootNormalForces() const
{
    Vec4 normal_forces = Vec4::Zero();

    for (int leg = 0; leg < 4; ++leg)
    {
        if (leg < static_cast<int>(ctrl_interfaces_.foot_force_state_interface_.size()))
        {
            normal_forces[leg] = ctrl_interfaces_.foot_force_state_interface_[leg].get().get_value();
        }
        if (!std::isfinite(normal_forces[leg]))
        {
            normal_forces[leg] = 0.0;
        }
        normal_forces[leg] = std::max(0.0, normal_forces[leg]);
    }

    return normal_forces;
}

void StateMPCWBCTrotting::updateDesiredCommand(const double dt, const RotMat &yaw_rotation_body_to_world)
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

Vec34 StateMPCWBCTrotting::buildCurrentFootLeversWorld(const RawBaseState &base_state) const
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

Vec34 StateMPCWBCTrotting::buildTouchdownPositionsWorld(const RotMat &yaw_rotation_body_to_world,
                                                        const Vec3 &desired_velocity_world,
                                                        const double desired_yaw_rate,
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

Vec12 StateMPCWBCTrotting::buildInitialMpcState(const RawBaseState &base_state) const
{
    Vec12 initial_state = Vec12::Zero();
    initial_state.segment<3>(0) = base_state.pos_world;
    initial_state.segment<3>(3) = base_state.rpy_world;
    initial_state.segment<3>(6) = base_state.linear_vel_world;
    initial_state.segment<3>(9) = base_state.angular_vel_world;
    return initial_state;
}

Vec34 StateMPCWBCTrotting::buildFallbackContactForces(const VecInt4 &mask) const
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

Vec34 StateMPCWBCTrotting::sanitizeContactForces(const Vec34 &forces_world, const VecInt4 &mask) const
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

Vec34 StateMPCWBCTrotting::enforceCurrentContactMask(const Vec34 &forces_world, const VecInt4 &mask) const
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

WbcInput StateMPCWBCTrotting::buildWbcInput(const RawBaseState &base_state, const VecInt4 &contact,
                                            const Vec34 &desired_swing_foot_acceleration_world)
{
    if (!ctrl_component_.robot_model_)
    {
        throw std::runtime_error("robot model is unavailable");
    }

    auto &pin_model = ctrl_component_.robot_model_->pinModel();
    const auto dynamics = pin_model.computeDynamicsTerms();

    WbcInput input;
    input.mass_matrix = dynamics.M;
    input.nonlinear_effects = dynamics.C * pin_model.dq() + dynamics.g;
    input.contact = contact;
    input.contact_activation = contact.cast<double>();
    input.mpc_contact_forces_world = last_contact_forces_world_;
    input.desired_base_acceleration_tangent = buildDesiredBaseAccelerationTangent(base_state);
    last_desired_base_acceleration_tangent_ = input.desired_base_acceleration_tangent;
    input.desired_joint_acceleration = buildDesiredJointAcceleration();
    input.desired_swing_foot_acceleration_world = desired_swing_foot_acceleration_world;

    input.friction_coefficient = CentroidalMPC::kFrictionCoefficient;
    input.min_normal_force = CentroidalMPC::kMinNormalForce * input.contact_activation;
    input.max_normal_force = CentroidalMPC::kMaxNormalForce * input.contact_activation;

    for (int leg = 0; leg < 4; ++leg)
    {
        input.foot_jacobians_world[leg] = pin_model.fullFootJacobianWorld(leg);
        input.foot_jdot_dq_world[leg] = pin_model.computeJdotDqWorld(leg);

        for (int joint = 0; joint < 3; ++joint)
        {
            const int index = 3 * leg + joint;
            const double limit = jointTorqueLimit(joint);
            input.torque_lower_bound[index] = -limit;
            input.torque_upper_bound[index] = limit;
        }
    }

    return input;
}

Vec6 StateMPCWBCTrotting::buildDesiredBaseAccelerationTangent(const RawBaseState &base_state) const
{
    Vec3 desired_position_world = desired_base_pos_world_;
    Vec3 desired_velocity_world = Vec3::Zero();
    Vec3 desired_rpy_world = Vec3::Zero();
    Vec3 desired_angular_velocity_world = Vec3::Zero();

    if (com_trajectory_.horizonSteps() > 0)
    {
        desired_position_world = com_trajectory_.desiredPositionWorld();
        desired_velocity_world = com_trajectory_.desiredVelocityWorld();
        desired_rpy_world = com_trajectory_.rpyTrajectoryWorld().col(0);
        desired_angular_velocity_world = com_trajectory_.omegaTrajectoryWorld().col(0);
    }
    else
    {
        desired_rpy_world.z() = base_state.rpy_world.z();
    }

    const Vec3 position_error_world = desired_position_world - base_state.pos_world;
    const Vec3 velocity_error_world = desired_velocity_world - base_state.linear_vel_world;

    Vec3 desired_linear_acceleration_world =
        kBasePositionKp * position_error_world + kBaseLinearVelocityKd * velocity_error_world;

    Vec3 orientation_error_world = Vec3::Zero();
    for (int axis = 0; axis < 3; ++axis)
    {
        const double raw_error = desired_rpy_world[axis] - base_state.rpy_world[axis];
        orientation_error_world[axis] = std::atan2(std::sin(raw_error), std::cos(raw_error));
    }

    Vec3 desired_angular_acceleration_world =
        kBaseOrientationKp * orientation_error_world +
        kBaseAngularVelocityKd * (desired_angular_velocity_world - base_state.angular_vel_world);

    for (int axis = 0; axis < 3; ++axis)
    {
        desired_linear_acceleration_world[axis] = std::clamp(desired_linear_acceleration_world[axis],
                                                             -kMaxBaseLinearAcceleration, kMaxBaseLinearAcceleration);
        desired_angular_acceleration_world[axis] = std::clamp(
            desired_angular_acceleration_world[axis], -kMaxBaseAngularAcceleration, kMaxBaseAngularAcceleration);
    }

    const Vec3 base_linear_velocity_body = base_state.rotation_body_to_world.transpose() * base_state.linear_vel_world;

    Vec6 desired_acceleration_tangent = Vec6::Zero();
    desired_acceleration_tangent.head<3>() =
        base_state.rotation_body_to_world.transpose() * desired_linear_acceleration_world -
        base_state.angular_vel_body.cross(base_linear_velocity_body);
    desired_acceleration_tangent.tail<3>() =
        base_state.rotation_body_to_world.transpose() * desired_angular_acceleration_world;

    return desired_acceleration_tangent;
}

Vec12 StateMPCWBCTrotting::buildDesiredJointAcceleration() const
{
    Vec12 desired_joint_acceleration = Vec12::Zero();
    if (!ctrl_component_.robot_model_)
    {
        return desired_joint_acceleration;
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            const int index = 3 * leg + joint;
            desired_joint_acceleration[index] =
                std::clamp(-kJointVelocityKd * ctrl_component_.robot_model_->current_joint_vel_[leg][joint],
                           -kMaxJointAcceleration, kMaxJointAcceleration);
        }
    }

    return desired_joint_acceleration;
}

StateMPCWBCTrotting::FootTrajectoryReference
StateMPCWBCTrotting::buildFootTrajectoryReference(const double gait_time, const VecInt4 &contact,
                                                  const Vec34 &touchdown_positions_world)
{
    Vec34 current_positions_world = Vec34::Zero();
    Vec34 current_velocities_world = Vec34::Zero();
    FootTrajectoryReference reference;

    auto &pin_model = ctrl_component_.robot_model_->pinModel();
    for (int leg = 0; leg < 4; ++leg)
    {
        current_positions_world.col(leg) = pin_model.footPositionWorld(leg);
        current_velocities_world.col(leg) = pin_model.footVelocityWorld(leg);
    }

    gait_.updateSwingTrajectory(gait_time, current_positions_world, touchdown_positions_world,
                                reference.position_world, reference.velocity_world, reference.acceleration_world);

    for (int leg = 0; leg < 4; ++leg)
    {
        if (contact[leg] == 1)
        {
            reference.acceleration_world.col(leg).setZero();
            continue;
        }

        reference.acceleration_world.col(leg) +=
            kSwingPositionKp * (reference.position_world.col(leg) - current_positions_world.col(leg)) +
            kSwingVelocityKd * (reference.velocity_world.col(leg) - current_velocities_world.col(leg));

        for (int axis = 0; axis < 3; ++axis)
        {
            reference.acceleration_world(axis, leg) =
                std::clamp(reference.acceleration_world(axis, leg), -kMaxSwingFootAcceleration,
                           kMaxSwingFootAcceleration);
        }
    }

    return reference;
}

StateMPCWBCTrotting::JointImpedanceReference
StateMPCWBCTrotting::buildJointImpedanceReference(const RawBaseState &base_state,
                                                 const FootTrajectoryReference &foot_reference) const
{
    JointImpedanceReference reference;
    if (!ctrl_component_.robot_model_)
    {
        return reference;
    }

    auto &pin_model = ctrl_component_.robot_model_->pinModel();
    const RotMat rotation_world_to_body = base_state.rotation_body_to_world.transpose();

    for (int leg = 0; leg < 4; ++leg)
    {
        Eigen::VectorXd current_leg_position(3);
        for (int joint = 0; joint < 3; ++joint)
        {
            const int index = 3 * leg + joint;
            current_leg_position[joint] = ctrl_component_.robot_model_->current_joint_pos_[leg][joint];
            reference.position[index] = current_leg_position[joint];
            reference.velocity[index] = ctrl_component_.robot_model_->current_joint_vel_[leg][joint];
        }

        const Vec3 foot_position_body = rotation_world_to_body *
                                        (foot_reference.position_world.col(leg) - base_state.pos_world);
        const Vec3 foot_velocity_body =
            rotation_world_to_body * (foot_reference.velocity_world.col(leg) - base_state.linear_vel_world) -
            base_state.angular_vel_body.cross(foot_position_body);

        const Eigen::VectorXd desired_leg_position =
            pin_model.solveLegIKBody(leg, foot_position_body, current_leg_position);

        const Mat3 joint_jacobian_body = pin_model.footJacobianBody(leg);
        Mat3 damped_jjt = joint_jacobian_body * joint_jacobian_body.transpose();
        damped_jjt.diagonal().array() += kJointVelocityInverseDamping;
        const Vec3 desired_leg_velocity =
            joint_jacobian_body.transpose() * damped_jjt.ldlt().solve(foot_velocity_body);

        if (desired_leg_position.allFinite() && desired_leg_velocity.allFinite())
        {
            reference.position.segment<3>(3 * leg) = desired_leg_position;
            reference.velocity.segment<3>(3 * leg) = desired_leg_velocity;
        }
    }

    return reference;
}

Mat3 StateMPCWBCTrotting::bodyInertiaWorld(const RotMat &base_rotation_body_to_world)
{
    const Mat3 body_inertia = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
    return base_rotation_body_to_world * body_inertia * base_rotation_body_to_world.transpose();
}

double StateMPCWBCTrotting::jointTorqueLimit(const int joint)
{
    return joint == 2 ? kCalfTorqueLimit : (joint == 1 ? kThighTorqueLimit : kHipTorqueLimit);
}

double StateMPCWBCTrotting::clampJointTorque(const double torque, const int joint)
{
    const double limit = jointTorqueLimit(joint);
    return std::clamp(std::isfinite(torque) ? torque : 0.0, -limit, limit);
}

bool StateMPCWBCTrotting::isBaseStateSafeForWbc(const RawBaseState &base_state) const
{
    return std::isfinite(base_state.pos_world.z()) && std::isfinite(base_state.rpy_world.x()) &&
           std::isfinite(base_state.rpy_world.y()) && base_state.pos_world.z() >= kMinWbcBaseHeight &&
           base_state.pos_world.z() <= kMaxWbcBaseHeight && std::abs(base_state.rpy_world.x()) <= kMaxWbcAbsRollPitch &&
           std::abs(base_state.rpy_world.y()) <= kMaxWbcAbsRollPitch;
}

Vec12 StateMPCWBCTrotting::selectCommandedTorques(const Vec12 &fallback_torques, const WbcOutput &wbc_output,
                                                  const VecInt4 &contact, const bool base_state_safe, const double dt)
{
    const bool wbc_eligible = wbc_output.success && wbc_output.joint_torques.allFinite() && base_state_safe;

    if (!wbc_eligible)
    {
        if (torque_authority_ != TorqueAuthority::FALLBACK)
        {
            ++wbc_fallback_event_count_;
            RCLCPP_WARN(rclcpp::get_logger("StateMPCWBCTrotting"),
                        "WBC authority revoked; LegController is now the torque target "
                        "(solver_ok=%d base_safe=%d)",
                        wbc_output.success, base_state_safe);
        }
        resetTorqueAuthority();
        return applyTorqueRateLimit(fallback_torques, dt);
    }

    ++consecutive_wbc_successes_;
    if (!has_authority_contact_)
    {
        last_authority_contact_ = contact;
        has_authority_contact_ = true;
    }
    else if ((contact.array() != last_authority_contact_.array()).any())
    {
        ++qualified_contact_switches_;
        last_authority_contact_ = contact;
    }

    if (torque_authority_ == TorqueAuthority::FALLBACK && consecutive_wbc_successes_ >= kRequiredWbcSuccessCycles &&
        qualified_contact_switches_ >= kRequiredWbcContactSwitches)
    {
        torque_authority_ = TorqueAuthority::BLENDING_TO_WBC;
        wbc_blend_ = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("StateMPCWBCTrotting"),
                    "WBC qualification passed after %zu consecutive solves and %zu contact switches; "
                    "starting %.2f s torque blend",
                    consecutive_wbc_successes_, qualified_contact_switches_, kWbcBlendDuration);
    }

    if (torque_authority_ == TorqueAuthority::BLENDING_TO_WBC)
    {
        const double safe_dt = std::clamp(std::isfinite(dt) ? dt : 0.0, 0.0, kMaxTorqueRateDt);
        wbc_blend_ = std::min(1.0, wbc_blend_ + safe_dt / kWbcBlendDuration);
        if (wbc_blend_ >= 1.0)
        {
            torque_authority_ = TorqueAuthority::WBC;
            RCLCPP_INFO(rclcpp::get_logger("StateMPCWBCTrotting"),
                        "WBC now has full torque authority after the guarded blend");
        }
    }

    Vec12 target_torques = fallback_torques;
    if (torque_authority_ != TorqueAuthority::FALLBACK)
    {
        target_torques = (1.0 - wbc_blend_) * fallback_torques + wbc_blend_ * wbc_output.joint_torques;
    }

    return applyTorqueRateLimit(target_torques, dt);
}

Vec12 StateMPCWBCTrotting::applyTorqueRateLimit(const Vec12 &target_torques, const double dt)
{
    Vec12 limited_torques = last_commanded_torques_;
    const double safe_dt = std::clamp(std::isfinite(dt) ? dt : 0.0, 0.0, kMaxTorqueRateDt);
    const double max_delta = kMaxTorqueRate * safe_dt;

    for (int index = 0; index < 12; ++index)
    {
        const int joint = index % 3;
        const double safe_target = clampJointTorque(target_torques[index], joint);
        const double delta = std::clamp(safe_target - last_commanded_torques_[index], -max_delta, max_delta);
        limited_torques[index] = clampJointTorque(last_commanded_torques_[index] + delta, joint);
    }

    last_commanded_torques_ = limited_torques;
    return limited_torques;
}

void StateMPCWBCTrotting::resetTorqueAuthority()
{
    torque_authority_ = TorqueAuthority::FALLBACK;
    consecutive_wbc_successes_ = 0;
    qualified_contact_switches_ = 0;
    last_authority_contact_.setOnes();
    has_authority_contact_ = false;
    wbc_blend_ = 0.0;
}

const char *StateMPCWBCTrotting::torqueAuthorityName() const
{
    switch (torque_authority_)
    {
    case TorqueAuthority::FALLBACK:
        return "fallback";
    case TorqueAuthority::BLENDING_TO_WBC:
        return "blending";
    case TorqueAuthority::WBC:
        return "wbc";
    }
    return "unknown";
}

void StateMPCWBCTrotting::commandHoldPosition()
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

void StateMPCWBCTrotting::commandHybridTorqueControl(const JointImpedanceReference &reference,
                                                     const VecInt4 &contact, const double gain_scale)
{
    const std::size_t joint_count = std::min<std::size_t>(12, ctrl_interfaces_.joint_torque_command_interface_.size());
    const double safe_gain_scale = std::clamp(std::isfinite(gain_scale) ? gain_scale : 0.0, 0.0, 1.0);

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        const int leg = static_cast<int>(i / 3);
        const bool swing = contact[leg] == 0;

        if (i < ctrl_interfaces_.joint_position_command_interface_.size())
        {
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(reference.position[i]);
        }
        if (i < ctrl_interfaces_.joint_velocity_command_interface_.size())
        {
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(reference.velocity[i]);
        }
        if (i < ctrl_interfaces_.joint_kp_command_interface_.size())
        {
            const double kp = swing ? kSwingJointKp : kStanceJointKp;
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(safe_gain_scale * kp);
        }
        if (i < ctrl_interfaces_.joint_kd_command_interface_.size())
        {
            const double kd = swing ? kSwingJointKd : kStanceJointKd;
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(safe_gain_scale * kd);
        }
    }
}

void StateMPCWBCTrotting::writeJointTorques(const Vec12 &joint_torques)
{
    const std::size_t joint_count = std::min<std::size_t>(12, ctrl_interfaces_.joint_torque_command_interface_.size());

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(joint_torques[static_cast<int>(i)]);
    }
}

void StateMPCWBCTrotting::commandZero()
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
