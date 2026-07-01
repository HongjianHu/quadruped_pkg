#ifndef QUADRUPED_CONTROLLER__MPC__COM_TRAJECTORY_H
#define QUADRUPED_CONTROLLER__MPC__COM_TRAJECTORY_H

#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/gait/Gait.h"
#include <array>
#include <vector>
namespace quadruped_controller
{

class ComTrajectory
{
  public:
    struct Input
    {
        Vec3 initial_pos_world = Vec3::Zero();
        Vec3 initial_rpy_world = Vec3::Zero();
        RotMat yaw_rotation_body_to_world = RotMat::Identity();

        Vec34 initial_foot_levers_world = Vec34::Zero();
        Vec34 hip_offsets_body = Vec34::Zero();

        double desired_x_vel_body = 0.0;
        double desired_y_vel_body = 0.0;
        double desired_z_pos_world = 0.0;
        double desired_yaw_rate_body = 0.0;
        double mass = 0.0;
        Mat3 inertia_com_world = Mat3::Identity();
        double horizon_time = 0.0;
        double time_now = 0.0;
        double time_step = 0.0;
    };

    explicit ComTrajectory(const Vec3 &initial_com_pos_world = Vec3::Zero());

    void resetDesiredPosition(const Vec3 &com_pos_world);
    void generateReference(const Input &input, const Gait &gait);

    MatX computeXRefVec() const;

    const MatX &posTrajectoryWorld() const;
    const MatX &rpyTrajectoryWorld() const;
    const MatX &velTrajectoryWorld() const;
    const MatX &omegaTrajectoryWorld() const;
    const Gait::ContactTable &contactTable() const;
    const MatX &footLeverTrajectoryWorld(int leg) const;
    const Mat12 &continuousA() const;
    const std::vector<Mat12> &continuousB() const;
    const Vec12 &continuousGravity() const;
    const Mat12 &discreteA() const;
    const std::vector<Mat12> &discreteB() const;
    const Vec12 &discreteGravity() const;
    const Vec3 &desiredPositionWorld() const;
    const Vec3 &desiredVelocityWorld() const;
    int horizonSteps() const;

  private:
    static Mat3 skew(const Vec3 &vector);
    void computeContinuousDynamics(const Input &input);
    void computeDiscreteDynamics(const Input &input);
    Vec3 pos_des_world_ = Vec3::Zero();
    Vec3 vel_des_world_ = Vec3::Zero();

    MatX pos_traj_world_;
    MatX rpy_traj_world_;
    MatX vel_traj_world_;
    MatX omega_traj_world_;
    Mat12 discrete_a_ = Mat12::Identity();
    std::vector<Mat12> discrete_b_;
    Vec12 discrete_g_ = Vec12::Zero();
    Mat12 continuous_a_ = Mat12::Zero();
    std::vector<Mat12> continuous_b_;
    Vec12 continuous_g_ = Vec12::Zero();

    Gait::ContactTable contact_table_;
    std::array<MatX, 4> foot_lever_traj_world_;
    int horizon_steps_ = 0;
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER__MPC__COM_TRAJECTORY_H