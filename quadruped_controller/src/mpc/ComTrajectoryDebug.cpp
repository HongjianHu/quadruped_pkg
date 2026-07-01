#include "quadruped_controller/mpc/ComTrajectory.h"

#include <cmath>
#include <iostream>

using quadruped_controller::ComTrajectory;
using quadruped_controller::Gait;
using quadruped_controller::Mat3;
using quadruped_controller::RotMat;
using quadruped_controller::Vec3;
using quadruped_controller::Vec34;
int main()
{
    Vec3 initial_pos_world;
    initial_pos_world << 0.0, 0.0, 0.30;

    Vec3 initial_rpy_world;
    initial_rpy_world << 0.0, 0.0, 0.2;

    const double yaw = initial_rpy_world.z();
    RotMat yaw_rotation_body_to_world;
    yaw_rotation_body_to_world << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;

    ComTrajectory traj(initial_pos_world);
    Gait gait(1.0 / 0.45, 0.5);
    ComTrajectory::Input input;
    Vec34 initial_foot_levers_world = Vec34::Zero();
    initial_foot_levers_world.col(0) << 0.2, 0.1, -0.30;
    initial_foot_levers_world.col(1) << 0.2, -0.1, -0.30;
    initial_foot_levers_world.col(2) << -0.2, 0.1, -0.30;
    initial_foot_levers_world.col(3) << -0.2, -0.1, -0.30;

    Vec34 hip_offsets_body = Vec34::Zero();
    hip_offsets_body.col(0) << 0.2, 0.1, 0.0;
    hip_offsets_body.col(1) << 0.2, -0.1, 0.0;
    hip_offsets_body.col(2) << -0.2, 0.1, 0.0;
    hip_offsets_body.col(3) << -0.2, -0.1, 0.0;

    input.initial_foot_levers_world = initial_foot_levers_world;
    input.hip_offsets_body = hip_offsets_body;
    input.initial_pos_world = initial_pos_world;
    input.initial_rpy_world = initial_rpy_world;
    input.yaw_rotation_body_to_world = yaw_rotation_body_to_world;

    input.desired_x_vel_body = 0.2;
    input.desired_y_vel_body = 0.1;
    input.desired_z_pos_world = 0.32;
    input.desired_yaw_rate_body = 0.3;
    input.mass = 15.0;
    input.inertia_com_world = Mat3::Zero();
    input.inertia_com_world.diagonal() << 0.06, 0.18, 0.20;
    input.horizon_time = 0.45;
    input.time_now = 0.0;
    input.time_step = 0.05;

    traj.generateReference(input, gait);

    const auto x_ref = traj.computeXRefVec();

    std::cout << "horizon steps: " << traj.horizonSteps() << "\n";
    std::cout << "x_ref rows: " << x_ref.rows() << ", cols: " << x_ref.cols() << "\n";

    std::cout << "desired pos world: " << traj.desiredPositionWorld().transpose() << "\n";
    std::cout << "desired vel world: " << traj.desiredVelocityWorld().transpose() << "\n";

    std::cout << "x_ref first column:\n" << x_ref.col(0).transpose() << "\n";
    std::cout << "x_ref last column:\n" << x_ref.col(x_ref.cols() - 1).transpose() << "\n";

    std::cout << "pos trajectory rows=[x y z], cols=horizon:\n" << traj.posTrajectoryWorld() << "\n";

    std::cout << "rpy trajectory rows=[roll pitch yaw], cols=horizon:\n" << traj.rpyTrajectoryWorld() << "\n";
    std::cout << "contact table rows=[FL FR RL RR], cols=horizon:\n" << traj.contactTable() << "\n";
    std::cout << "foot lever FL world rows=[x y z], cols=horizon:\n" << traj.footLeverTrajectoryWorld(0) << "\n";
    std::cout << "foot lever FR world rows=[x y z], cols=horizon:\n" << traj.footLeverTrajectoryWorld(1) << "\n";
    std::cout << "foot lever RL world rows=[x y z], cols=horizon:\n" << traj.footLeverTrajectoryWorld(2) << "\n";
    std::cout << "foot lever RR world rows=[x y z], cols=horizon:\n" << traj.footLeverTrajectoryWorld(3) << "\n";
    std::cout << "Ac:\n" << traj.continuousA() << "\n";
    std::cout << "gc: " << traj.continuousGravity().transpose() << "\n";

    std::cout << "Bc[0] force block rows=[vx vy vz], cols=[FL FR RL RR forces]:\n"
              << traj.continuousB().front().block<3, 12>(6, 0) << "\n";

    std::cout << "Bc[0] angular block rows=[wx wy wz], cols=[FL FR RL RR forces]:\n"
              << traj.continuousB().front().block<3, 12>(9, 0) << "\n";
    std::cout << "Ad:\n" << traj.discreteA() << "\n";
    std::cout << "gd: " << traj.discreteGravity().transpose() << "\n";

    std::cout << "Bd[0] position block rows=[px py pz], cols=[FL FR RL RR forces]:\n"
              << traj.discreteB().front().block<3, 12>(0, 0) << "\n";

    std::cout << "Bd[0] velocity block rows=[vx vy vz], cols=[FL FR RL RR forces]:\n"
              << traj.discreteB().front().block<3, 12>(6, 0) << "\n";

    std::cout << "Bd[0] rpy block rows=[roll pitch yaw], cols=[FL FR RL RR forces]:\n"
              << traj.discreteB().front().block<3, 12>(3, 0) << "\n";

    std::cout << "Bd[0] omega block rows=[wx wy wz], cols=[FL FR RL RR forces]:\n"
              << traj.discreteB().front().block<3, 12>(9, 0) << "\n";
    return 0;
}