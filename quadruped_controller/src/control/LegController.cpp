#include "quadruped_controller/control/LegController.h"

#include <stdexcept>

namespace quadruped_controller
{

LegController::LegController()
{
    last_mask_ = VecInt4::Constant(2);
    takeoff_time_.fill(0.0);
    has_swing_plan_.fill(false);
    has_touchdown_target_.fill(false);

    for (auto &pos : takeoff_pos_world_)
    {
        pos.setZero();
    }

    for (auto &pos : touchdown_pos_world_)
    {
        pos.setZero();
    }
}

void LegController::setTouchdownPositionWorld(const int leg, const Vec3 &position)
{
    if (leg < 0 || leg >= 4)
    {
        throw std::runtime_error("LegController leg index out of range");
    }

    touchdown_pos_world_[leg] = position;
    has_touchdown_target_[leg] = true;
}

void LegController::setTouchdownPositionsWorld(const Vec34 &positions)
{
    for (int leg = 0; leg < 4; ++leg)
    {
        setTouchdownPositionWorld(leg, positions.col(leg));
    }
}

int LegController::legIndex(const std::string &leg_name)
{
    if (leg_name == "FL")
    {
        return 0;
    }
    if (leg_name == "FR")
    {
        return 1;
    }
    if (leg_name == "RL")
    {
        return 2;
    }
    if (leg_name == "RR")
    {
        return 3;
    }

    throw std::runtime_error("Unknown leg name: " + leg_name);
}

Mat3 LegController::swingKp()
{
    Mat3 kp = Mat3::Zero();
    kp.diagonal() << 400.0, 400.0, 400.0;
    return kp;
}

Mat3 LegController::swingKd()
{
    Mat3 kd = Mat3::Zero();
    kd.diagonal() << 75.0, 75.0, 75.0;
    return kd;
}

LegOutput LegController::computeLegTorque(const std::string &leg_name, go2_robot_data::PinGo2Model &go2, Gait &gait,
                                          const Vec3 &contact_force_world, const double current_time)
{
    return computeLegTorque(legIndex(leg_name), go2, gait, contact_force_world, current_time);
}

LegOutput LegController::computeLegTorque(const int leg, go2_robot_data::PinGo2Model &go2, Gait &gait,
                                          const Vec3 &contact_force_world, const double current_time)
{
    if (leg < 0 || leg >= 4)
    {
        throw std::runtime_error("LegController leg index out of range");
    }

    const auto foot_state = go2.footStateWorld(leg);
    const VecInt4 current_mask = gait.computeCurrentMask(current_time);

    LegOutput output;
    output.tau.setZero();
    output.pos_now = foot_state.position;
    output.vel_now = foot_state.velocity;
    output.pos_des = foot_state.position;
    output.vel_des = foot_state.velocity;

    if (last_mask_[leg] != current_mask[leg] && current_mask[leg] == 0)
    {
        takeoff_time_[leg] = current_time;
        takeoff_pos_world_[leg] = foot_state.position;

        if (!has_touchdown_target_[leg])
        {
            touchdown_pos_world_[leg] = foot_state.position;
        }

        has_swing_plan_[leg] = true;
    }

    if (current_mask[leg] == 0 && has_swing_plan_[leg])
    {
        const double time_since_takeoff = current_time - takeoff_time_[leg];

        const auto point = Gait::evaluateSwingTrajectory(takeoff_pos_world_[leg], touchdown_pos_world_[leg],
                                                         time_since_takeoff, gait.swingTime(), gait.swingHeight());

        output.pos_des = point.position;
        output.vel_des = point.velocity;

        const Vec3 pos_error = output.pos_des - output.pos_now;
        const Vec3 vel_error = output.vel_des - output.vel_now;

        const Mat3 J_foot_world = go2.footJacobianWorld(leg);
        const Eigen::MatrixXd J_full_foot_world = go2.fullFootJacobianWorld(leg);

        const auto dynamics = go2.computeDynamicsTerms();

        Eigen::LDLT<MatX> mass_ldlt(dynamics.M);
        const MatX minv_jt = mass_ldlt.solve(J_full_foot_world.transpose());

        Mat3 lambda_inv = J_full_foot_world * minv_jt;
        lambda_inv.diagonal().array() += 1.0e-6;

        const Mat3 Lambda = lambda_inv.ldlt().solve(Mat3::Identity());
        const Vec3 Jdot_dq = go2.computeJdotDqWorld(leg);

        const Vec3 f_ff = Lambda * (point.acceleration - Jdot_dq);
        const Vec3 force = swingKp() * pos_error + swingKd() * vel_error + f_ff;

        const VecX bias = dynamics.C * go2.dq() + dynamics.g;
        output.tau = J_foot_world.transpose() * force + go2.legJointVector(bias, leg);
    }

    if (current_mask[leg] == 1)
    {
        has_swing_plan_[leg] = false;
        const Mat3 J_foot_world = go2.footJacobianWorld(leg);
        output.tau = J_foot_world.transpose() * (-contact_force_world);
    }

    last_mask_[leg] = current_mask[leg];

    return output;
}

} // namespace quadruped_controller