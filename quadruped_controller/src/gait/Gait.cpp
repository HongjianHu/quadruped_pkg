#include "quadruped_controller/gait/Gait.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quadruped_controller
{

Vec4 Gait::defaultPhaseOffset()
{
    Vec4 offset;
    offset << 0.0, 0.5, 0.5, 0.0;
    return offset;
}

double Gait::defaultSwingHeight()
{
    return 0.1;
}

Gait::SwingTrajectoryPoint Gait::evaluateSwingTrajectory(const Vec3 &start_pos, const Vec3 &end_pos, const double time,
                                                         const double swing_time)
{
    return evaluateSwingTrajectory(start_pos, end_pos, time, swing_time, defaultSwingHeight());
}

Gait::SwingTrajectoryPoint Gait::evaluateSwingTrajectory(const Vec3 &start_pos, const Vec3 &end_pos, const double time,
                                                         const double swing_time, const double swing_height)
{
    if (swing_time <= 0.0)
    {
        throw std::invalid_argument("Gait swing_time must be positive");
    }

    const double s = std::clamp(time / swing_time, 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;

    const double mj = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    const double dmj = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
    const double d2mj = 60.0 * s - 180.0 * s2 + 120.0 * s3;

    const Vec3 dp = end_pos - start_pos;

    SwingTrajectoryPoint point;
    point.position = start_pos + dp * mj;
    point.velocity = dp * dmj / swing_time;
    point.acceleration = dp * d2mj / (swing_time * swing_time);

    if (swing_height != 0.0)
    {
        const double one_minus_s = 1.0 - s;
        const double b = 64.0 * s3 * one_minus_s * one_minus_s * one_minus_s;
        const double db = 192.0 * s2 * one_minus_s * one_minus_s * (1.0 - 2.0 * s);
        const double d2b = 192.0 * (2.0 * s * one_minus_s * one_minus_s * (1.0 - 2.0 * s) -
                                    2.0 * s2 * one_minus_s * (1.0 - 2.0 * s) - 2.0 * s2 * one_minus_s * one_minus_s);

        point.position.z() += swing_height * b;
        point.velocity.z() += swing_height * db / swing_time;
        point.acceleration.z() += swing_height * d2b / (swing_time * swing_time);
    }

    return point;
}

Gait::Gait(const double frequency_hz, const double duty) : Gait(frequency_hz, duty, defaultPhaseOffset())
{
}

Gait::Gait(const double frequency_hz, const double duty, const Vec4 &phase_offset)
    : gait_duty_(duty), gait_hz_(frequency_hz), phase_offset_(phase_offset)
{
    if (gait_hz_ <= 0.0)
    {
        throw std::invalid_argument("Gait frequency_hz must be positive");
    }

    if (gait_duty_ <= 0.0 || gait_duty_ >= 1.0)
    {
        throw std::invalid_argument("Gait duty must be in (0, 1)");
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        if (phase_offset_[leg] < 0.0 || phase_offset_[leg] >= 1.0)
        {
            throw std::invalid_argument("Gait phase offset must be in [0, 1)");
        }
    }

    gait_period_ = 1.0 / gait_hz_;
    stance_time_ = gait_duty_ * gait_period_;
    swing_time_ = (1.0 - gait_duty_) * gait_period_;
}

Vec4 Gait::computePhase(const double time) const
{
    Vec4 phase;

    for (int leg = 0; leg < 4; ++leg)
    {
        phase[leg] = std::fmod(phase_offset_[leg] + time / gait_period_, 1.0);
        if (phase[leg] < 0.0)
        {
            phase[leg] += 1.0;
        }
    }

    return phase;
}

void Gait::setSwingHeight(const double swing_height)
{
    if (swing_height < 0.0)
    {
        throw std::invalid_argument("Gait swing height must be non-negative");
    }

    swing_height_ = swing_height;
}

void Gait::resetSwingState(const Vec34 &foot_positions)
{
    for (int leg = 0; leg < 4; ++leg)
    {
        swing_states_[leg].active = false;
        swing_states_[leg].start_pos = foot_positions.col(leg);
        swing_states_[leg].end_pos = foot_positions.col(leg);
    }
}

Vec3 Gait::computeTouchdownWorld(const TouchdownInput &input) const
{
    const double T = swing_time_ + 0.5 * stance_time_; // T 是从当前摆动腿开始到下一次支撑相中点的大致时间
    const double pred_time = 0.5 * T;

    const Vec3 body_pos_world(input.base_pos_world.x(), input.base_pos_world.y(), 0.0);

    const Vec3 hip_pos_world = body_pos_world + input.yaw_rotation_body_to_world * input.hip_offset_body;

    Vec3 nominal_pos = Vec3::Zero();

    nominal_pos << hip_pos_world.x(), hip_pos_world.y(), 0.02;

    const double k_v_x = 0.4 * T;
    const double k_p_x = 0.1;
    const double k_v_y = 0.2 * T;
    const double k_p_y = 0.05;

    Vec3 drift_term = Vec3::Zero();
    drift_term.x() = input.desired_velocity_world.x() * pred_time;
    drift_term.y() = input.desired_velocity_world.y() * pred_time;

    Vec3 pos_correction_term = Vec3::Zero();
    pos_correction_term.x() = k_p_x * (input.com_pos_world.x() - input.desired_position_world.x());
    pos_correction_term.y() = k_p_y * (input.com_pos_world.y() - input.desired_position_world.y());

    Vec3 vel_correction_term = Vec3::Zero();
    vel_correction_term.x() = k_v_x * (input.com_vel_world.x() - input.desired_velocity_world.x());
    vel_correction_term.y() = k_v_y * (input.com_vel_world.y() - input.desired_velocity_world.y());

    const double dtheta = input.yaw_rate_des_world * pred_time;
    const Eigen::Vector2d r_xy = nominal_pos.head<2>() - input.base_pos_world.head<2>();

    Vec3 rotation_correction_term = Vec3::Zero();
    rotation_correction_term.x() = -dtheta * r_xy.y();
    rotation_correction_term.y() = dtheta * r_xy.x();

    return nominal_pos + drift_term + pos_correction_term + vel_correction_term + rotation_correction_term;
}

Vec34 Gait::computeTouchdownWorlds(const TouchdownInput &input, const Vec34 &hip_offsets_body) const
{
    Vec34 touchdown_positions = Vec34::Zero();

    for (int leg = 0; leg < 4; ++leg)
    {
        TouchdownInput leg_input = input;
        leg_input.hip_offset_body = hip_offsets_body.col(leg);
        touchdown_positions.col(leg) = computeTouchdownWorld(leg_input);
    }

    return touchdown_positions;
}
// debug
VecInt4 Gait::computeCurrentMask(const double time) const
{
    const ContactTable table = computeContactTable(time, 0.0, 1);
    return table.col(0);
}

Gait::ContactTable Gait::computeContactTable(const double t0, const double dt, const int horizon) const
{
    if (horizon <= 0)
    {
        throw std::invalid_argument("Gait contact table horizon must be positive");
    }

    ContactTable contact_table(4, horizon);

    for (int k = 0; k < horizon; ++k)
    {
        const double sample_time = t0 + static_cast<double>(k) * dt + 0.5 * dt;

        for (int leg = 0; leg < 4; ++leg)
        {
            double phase = std::fmod(phase_offset_[leg] + sample_time / gait_period_, 1.0);
            if (phase < 0.0)
            {
                phase += 1.0;
            }

            contact_table(leg, k) = phase < gait_duty_ ? 1 : 0;
        }
    }

    return contact_table;
}

void Gait::updateSwingTrajectory(const double time, const Vec34 &current_foot_positions,
                                 const Vec34 &touchdown_positions, Vec34 &target_positions, Vec34 &target_velocities,
                                 Vec34 &target_accelerations)
{
    const VecInt4 contact = computeCurrentMask(time);
    const Vec4 phase = computePhase(time);

    for (int leg = 0; leg < 4; ++leg)
    {
        if (contact[leg] == 1)
        {
            swing_states_[leg].active = false;
            swing_states_[leg].start_pos = current_foot_positions.col(leg);
            swing_states_[leg].end_pos = current_foot_positions.col(leg);

            target_positions.col(leg) = current_foot_positions.col(leg);
            target_velocities.col(leg).setZero();
            target_accelerations.col(leg).setZero();
            continue;
        }

        if (!swing_states_[leg].active)
        {
            swing_states_[leg].active = true;
            swing_states_[leg].start_pos = current_foot_positions.col(leg);
            swing_states_[leg].end_pos = touchdown_positions.col(leg);
        }

        const double swing_phase = std::clamp((phase[leg] - gait_duty_) / (1.0 - gait_duty_), 0.0, 1.0);
        const double swing_elapsed = swing_phase * swing_time_;

        const SwingTrajectoryPoint point = evaluateSwingTrajectory(
            swing_states_[leg].start_pos, swing_states_[leg].end_pos, swing_elapsed, swing_time_, swing_height_);

        target_positions.col(leg) = point.position;
        target_velocities.col(leg) = point.velocity;
        target_accelerations.col(leg) = point.acceleration;
    }
}

} // namespace quadruped_controller
