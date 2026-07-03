#include "quadruped_controller/mpc/ComTrajectory.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace quadruped_controller
{

ComTrajectory::ComTrajectory(const Vec3 &initial_com_pos_world) : pos_des_world_(initial_com_pos_world)
{
}

void ComTrajectory::resetDesiredPosition(const Vec3 &com_pos_world)
{
    pos_des_world_ = com_pos_world;
}

Mat3 ComTrajectory::skew(const Vec3 &vector)
{
    Mat3 result;
    result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(), 0.0;
    return result;
}

void ComTrajectory::generateReference(const Input &input, const Gait &gait)
{
    if (input.time_step <= 0.0)
    {
        throw std::runtime_error("ComTrajectory time_step must be positive");
    }
    if (input.horizon_time <= 0.0)
    {
        throw std::runtime_error("ComTrajectory horizon_time must be positive");
    }

    horizon_steps_ = static_cast<int>(std::round(input.horizon_time / input.time_step));
    if (horizon_steps_ <= 0)
    {
        throw std::runtime_error("ComTrajectory horizon_steps must be positive");
    }

    const double max_pos_error = 0.1;

    const double x0 = input.initial_pos_world.x();
    const double y0 = input.initial_pos_world.y();

    pos_des_world_.x() = std::clamp(pos_des_world_.x(), x0 - max_pos_error, x0 + max_pos_error);
    pos_des_world_.y() = std::clamp(pos_des_world_.y(), y0 - max_pos_error, y0 + max_pos_error);
    pos_des_world_.z() = input.desired_z_pos_world;

    Vec3 vel_des_body = Vec3::Zero();
    vel_des_body << input.desired_x_vel_body, input.desired_y_vel_body, 0.0;
    vel_des_world_ = input.yaw_rotation_body_to_world * vel_des_body;

    pos_traj_world_ = MatX::Zero(3, horizon_steps_);
    rpy_traj_world_ = MatX::Zero(3, horizon_steps_);
    vel_traj_world_ = MatX::Zero(3, horizon_steps_);
    omega_traj_world_ = MatX::Zero(3, horizon_steps_);
    for (auto &foot_lever_traj : foot_lever_traj_world_)
    {
        foot_lever_traj = MatX::Zero(3, horizon_steps_);
    }
    const double yaw0 = input.initial_rpy_world.z();

    for (int i = 0; i < horizon_steps_; ++i)
    {
        const double t = static_cast<double>(i + 1) * input.time_step;

        pos_traj_world_.col(i) = pos_des_world_ + vel_des_world_ * t;
        vel_traj_world_.col(i) = vel_des_world_;

        rpy_traj_world_.col(i) << 0.0, 0.0, yaw0 + input.desired_yaw_rate_body * t;
        omega_traj_world_.col(i) << 0.0, 0.0, input.desired_yaw_rate_body;
    }

    contact_table_ = gait.computeContactTable(input.time_now, input.time_step, horizon_steps_);

    std::array<Vec3, 4> next_touchdown_levers_world;
    for (int leg = 0; leg < 4; ++leg)
    {
        next_touchdown_levers_world[leg] = input.initial_foot_levers_world.col(leg);
    }

    VecInt4 mask_previous;
    mask_previous.setConstant(-1);

    for (int i = 0; i < horizon_steps_; ++i)
    {
        const VecInt4 current_mask = contact_table_.col(i);

        const double yaw = rpy_traj_world_(2, i);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        RotMat yaw_rotation_body_to_world;
        yaw_rotation_body_to_world << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;

        Gait::TouchdownInput touchdown_input;
        touchdown_input.base_pos_world = pos_traj_world_.col(i);
        touchdown_input.com_pos_world = pos_traj_world_.col(i);
        touchdown_input.com_vel_world = vel_traj_world_.col(i);
        touchdown_input.yaw_rotation_body_to_world = yaw_rotation_body_to_world;
        touchdown_input.desired_velocity_world = vel_des_world_;
        touchdown_input.desired_position_world = pos_des_world_;
        touchdown_input.yaw_rate_des_world = input.desired_yaw_rate_body;

        const Vec34 touchdown_positions_world = gait.computeTouchdownWorlds(touchdown_input, input.hip_offsets_body);

        for (int leg = 0; leg < 4; ++leg)
        {
            if (current_mask[leg] != mask_previous[leg] && current_mask[leg] == 0)
            {
                next_touchdown_levers_world[leg] = touchdown_positions_world.col(leg) - touchdown_input.base_pos_world;
                foot_lever_traj_world_[leg].col(i).setZero();
            }
            else if (current_mask[leg] != mask_previous[leg] && current_mask[leg] == 1)
            {
                foot_lever_traj_world_[leg].col(i) = next_touchdown_levers_world[leg];
            }
            else if (i > 0)
            {
                foot_lever_traj_world_[leg].col(i) = foot_lever_traj_world_[leg].col(i - 1);
            }
            else
            {
                foot_lever_traj_world_[leg].col(i) =
                    current_mask[leg] == 1 ? next_touchdown_levers_world[leg] : Vec3::Zero();
            }
        }

        mask_previous = current_mask;
    }
    computeContinuousDynamics(input);
    computeDiscreteDynamics(input);
}

void ComTrajectory::computeContinuousDynamics(const Input &input)
{
    if (input.mass <= 0.0)
    {
        throw std::runtime_error("ComTrajectory mass must be positive");
    }

    if (std::abs(input.inertia_com_world.determinant()) < 1e-9)
    {
        throw std::runtime_error("ComTrajectory inertia_com_world must be invertible");
    }

    const Mat3 inertia_inv = input.inertia_com_world.inverse();

    const double yaw_avg = rpy_traj_world_.row(2).mean();
    const double cy = std::cos(yaw_avg);
    const double sy = std::sin(yaw_avg);

    RotMat yaw_rotation;
    yaw_rotation << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;

    continuous_a_ = Mat12::Zero();
    continuous_a_.block<3, 3>(0, 6) = Mat3::Identity();
    continuous_a_.block<3, 3>(3, 9) = yaw_rotation.transpose();

    continuous_b_.assign(static_cast<std::size_t>(horizon_steps_), Mat12::Zero());

    for (int i = 0; i < horizon_steps_; ++i)
    {
        Mat12 bi = Mat12::Zero();

        for (int leg = 0; leg < 4; ++leg)
        {
            const Vec3 r_world = foot_lever_traj_world_[leg].col(i);
            const Mat3 angular_block = inertia_inv * skew(r_world);

            bi.block<3, 3>(6, 3 * leg) = (1.0 / input.mass) * Mat3::Identity();
            bi.block<3, 3>(9, 3 * leg) = angular_block;
        }

        continuous_b_[static_cast<std::size_t>(i)] = bi;
    }

    continuous_g_ = Vec12::Zero();
    continuous_g_.segment<3>(6) << 0.0, 0.0, -9.81;
}

void ComTrajectory::computeDiscreteDynamics(const Input &input)
{
    const double dt = input.time_step;

    if (input.mass <= 0.0)
    {
        throw std::runtime_error("ComTrajectory mass must be positive");
    }

    if (std::abs(input.inertia_com_world.determinant()) < 1e-9)
    {
        throw std::runtime_error("ComTrajectory inertia_com_world must be invertible");
    }

    const Mat3 inertia_inv = input.inertia_com_world.inverse();

    const double yaw_avg = rpy_traj_world_.row(2).mean();
    const double cy = std::cos(yaw_avg);
    const double sy = std::sin(yaw_avg);

    RotMat yaw_rotation;
    yaw_rotation << cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0;

    const Mat3 rz_t = yaw_rotation.transpose();

    discrete_a_ = Mat12::Identity();
    discrete_a_.block<3, 3>(0, 6) = dt * Mat3::Identity();
    discrete_a_.block<3, 3>(3, 9) = dt * rz_t;

    const Vec3 gravity(0.0, 0.0, -9.81);
    discrete_g_ = Vec12::Zero();
    discrete_g_.segment<3>(0) = 0.5 * gravity * dt * dt;
    discrete_g_.segment<3>(6) = gravity * dt;

    discrete_b_.assign(static_cast<std::size_t>(horizon_steps_), Mat12::Zero());

    const Mat3 bp = (0.5 * dt * dt / input.mass) * Mat3::Identity();
    const Mat3 bv = (dt / input.mass) * Mat3::Identity();

    for (int i = 0; i < horizon_steps_; ++i)
    {
        Mat12 bi = Mat12::Zero();

        for (int leg = 0; leg < 4; ++leg)
        {
            const Vec3 r_world = foot_lever_traj_world_[leg].col(i);
            const Mat3 w = inertia_inv * skew(r_world);

            bi.block<3, 3>(0, 3 * leg) = bp;
            bi.block<3, 3>(6, 3 * leg) = bv;
            bi.block<3, 3>(9, 3 * leg) = dt * w;
            bi.block<3, 3>(3, 3 * leg) = 0.5 * dt * dt * (rz_t * w);
        }

        discrete_b_[static_cast<std::size_t>(i)] = bi;
    }
}

MatX ComTrajectory::computeXRefVec() const
{
    MatX ref = MatX::Zero(12, horizon_steps_);

    if (horizon_steps_ <= 0)
    {
        return ref;
    }

    ref.block(0, 0, 3, horizon_steps_) = pos_traj_world_;
    ref.block(3, 0, 3, horizon_steps_) = rpy_traj_world_;
    ref.block(6, 0, 3, horizon_steps_) = vel_traj_world_;
    ref.block(9, 0, 3, horizon_steps_) = omega_traj_world_;

    return ref;
}

const MatX &ComTrajectory::footLeverTrajectoryWorld(int leg) const
{
    if (leg < 0 || leg >= 4)
    {
        throw std::runtime_error("ComTrajectory foot lever leg index out of range");
    }

    return foot_lever_traj_world_[leg];
}

const MatX &ComTrajectory::posTrajectoryWorld() const
{
    return pos_traj_world_;
}

const MatX &ComTrajectory::rpyTrajectoryWorld() const
{
    return rpy_traj_world_;
}

const MatX &ComTrajectory::velTrajectoryWorld() const
{
    return vel_traj_world_;
}

const MatX &ComTrajectory::omegaTrajectoryWorld() const
{
    return omega_traj_world_;
}

const Vec3 &ComTrajectory::desiredPositionWorld() const
{
    return pos_des_world_;
}

const Vec3 &ComTrajectory::desiredVelocityWorld() const
{
    return vel_des_world_;
}

int ComTrajectory::horizonSteps() const
{
    return horizon_steps_;
}

const Gait::ContactTable &ComTrajectory::contactTable() const
{
    return contact_table_;
}

const Mat12 &ComTrajectory::continuousA() const
{
    return continuous_a_;
}

const std::vector<Mat12> &ComTrajectory::continuousB() const
{
    return continuous_b_;
}

const Vec12 &ComTrajectory::continuousGravity() const
{
    return continuous_g_;
}

const Mat12 &ComTrajectory::discreteA() const
{
    return discrete_a_;
}

const std::vector<Mat12> &ComTrajectory::discreteB() const
{
    return discrete_b_;
}

const Vec12 &ComTrajectory::discreteGravity() const
{
    return discrete_g_;
}
} // namespace quadruped_controller
