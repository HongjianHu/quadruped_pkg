#include "quadruped_controller/gait/Gait.h"
#include "quadruped_controller/mpc/CentroidalMPC.h"
#include "quadruped_controller/mpc/ComTrajectory.h"

#include <cmath>
#include <iostream>

using quadruped_controller::CentroidalMPC;
using quadruped_controller::ComTrajectory;
using quadruped_controller::Gait;
using quadruped_controller::Mat3;
using quadruped_controller::RotMat;
using quadruped_controller::Vec12;
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

    ComTrajectory traj(initial_pos_world);
    Gait gait(1.0 / 0.45, 0.5);

    ComTrajectory::Input input;
    input.initial_pos_world = initial_pos_world;
    input.initial_rpy_world = initial_rpy_world;
    input.yaw_rotation_body_to_world = yaw_rotation_body_to_world;
    input.initial_foot_levers_world = initial_foot_levers_world;
    input.hip_offsets_body = hip_offsets_body;
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

    CentroidalMPC mpc(traj.horizonSteps());
    const auto gradient = mpc.buildGradient(traj);
    const auto variable_lower = mpc.buildVariableLowerBound(traj);
    const auto variable_upper = mpc.buildVariableUpperBound(traj);
    const auto friction_lower = mpc.buildFrictionLowerBound();
    const auto friction_upper = mpc.buildFrictionUpperBound(traj);
    const auto dynamics_matrix = mpc.buildDynamicsMatrix(traj);
    Vec12 initial_state = Vec12::Zero();
    initial_state.segment<3>(0) = initial_pos_world;
    initial_state.segment<3>(3) = initial_rpy_world;

    const auto dynamics_bound = mpc.buildDynamicsBound(traj, initial_state);
    const auto constraint_matrix = mpc.buildConstraintMatrix(traj);
    const auto constraint_lower = mpc.buildConstraintLowerBound(traj, initial_state);
    const auto constraint_upper = mpc.buildConstraintUpperBound(traj, initial_state);
    const auto solver_matrix = mpc.buildSolverConstraintMatrix(traj);
    const auto solver_lower = mpc.buildSolverLowerBound(traj, initial_state);
    const auto solver_upper = mpc.buildSolverUpperBound(traj, initial_state);
    const auto solution = mpc.solveOnce(traj, initial_state);
    const auto first_force = mpc.extractFirstForce(solution);
    const auto solved_first_force = mpc.solveFirstForce(traj, initial_state);

    std::cout << "horizon steps: " << mpc.horizonSteps() << "\n";
    std::cout << "num variables: " << mpc.numVariables() << "\n";
    std::cout << "H rows: " << mpc.hessian().rows() << ", cols: " << mpc.hessian().cols()
              << ", nnz: " << mpc.hessian().nonZeros() << "\n";

    std::cout << "H first state diagonal block:\n";
    for (int i = 0; i < CentroidalMPC::kStateDim; ++i)
    {
        std::cout << mpc.hessian().coeff(i, i) << " ";
    }
    std::cout << "\n";

    std::cout << "H first input diagonal block:\n";
    const int input0 = mpc.inputStartIndex(0);
    for (int i = 0; i < CentroidalMPC::kInputDim; ++i)
    {
        std::cout << mpc.hessian().coeff(input0 + i, input0 + i) << " ";
    }
    std::cout << "\n";

    std::cout << "gradient head state block:\n" << gradient.segment(0, CentroidalMPC::kStateDim).transpose() << "\n";

    std::cout << "gradient first input block norm: " << gradient.segment(input0, CentroidalMPC::kInputDim).norm()
              << "\n";

    std::cout << "friction rows: " << mpc.frictionMatrix().rows() << ", cols: " << mpc.frictionMatrix().cols()
              << ", nnz: " << mpc.frictionMatrix().nonZeros() << "\n";

    std::cout << "first FL friction block [fx fy fz columns]:\n";
    for (int row = 0; row < 4; ++row)
    {
        std::cout << mpc.frictionMatrix().coeff(row, input0 + 0) << " " << mpc.frictionMatrix().coeff(row, input0 + 1)
                  << " " << mpc.frictionMatrix().coeff(row, input0 + 2) << "\n";
    }

    std::cout << "force bounds k=0 lower:\n"
              << variable_lower.segment(input0, CentroidalMPC::kInputDim).transpose() << "\n";
    std::cout << "force bounds k=0 upper:\n"
              << variable_upper.segment(input0, CentroidalMPC::kInputDim).transpose() << "\n";

    const int input4 = mpc.inputStartIndex(4);
    std::cout << "force bounds k=4 lower:\n"
              << variable_lower.segment(input4, CentroidalMPC::kInputDim).transpose() << "\n";
    std::cout << "force bounds k=4 upper:\n"
              << variable_upper.segment(input4, CentroidalMPC::kInputDim).transpose() << "\n";

    std::cout << "friction bounds k=0 lower:\n" << friction_lower.segment(0, 16).transpose() << "\n";
    std::cout << "friction bounds k=0 upper:\n" << friction_upper.segment(0, 16).transpose() << "\n";

    const int friction4 = 16 * 4;
    std::cout << "friction bounds k=4 lower:\n" << friction_lower.segment(friction4, 16).transpose() << "\n";
    std::cout << "friction bounds k=4 upper:\n" << friction_upper.segment(friction4, 16).transpose() << "\n";

    std::cout << "dynamics rows: " << dynamics_matrix.rows() << ", cols: " << dynamics_matrix.cols()
              << ", nnz: " << dynamics_matrix.nonZeros() << "\n";

    std::cout << "Aeq first row x0 coefficient: " << dynamics_matrix.coeff(0, mpc.stateStartIndex(0)) << "\n";

    std::cout << "Aeq second knot previous-state px coefficient: "
              << dynamics_matrix.coeff(CentroidalMPC::kStateDim, mpc.stateStartIndex(0)) << "\n";

    std::cout << "Aeq first knot first input px coefficient: " << dynamics_matrix.coeff(0, mpc.inputStartIndex(0))
              << "\n";

    std::cout << "dynamics bound first block:\n"
              << dynamics_bound.segment(0, CentroidalMPC::kStateDim).transpose() << "\n";

    std::cout << "dynamics bound second block:\n"
              << dynamics_bound.segment(CentroidalMPC::kStateDim, CentroidalMPC::kStateDim).transpose() << "\n";

    std::cout << "constraint rows: " << constraint_matrix.rows() << ", cols: " << constraint_matrix.cols()
              << ", nnz: " << constraint_matrix.nonZeros() << "\n";

    std::cout << "constraint lower first dynamics block:\n"
              << constraint_lower.segment(0, CentroidalMPC::kStateDim).transpose() << "\n";

    std::cout << "constraint upper first dynamics block:\n"
              << constraint_upper.segment(0, CentroidalMPC::kStateDim).transpose() << "\n";

    std::cout << "constraint lower first friction block:\n"
              << constraint_lower.segment(mpc.numDynamicsConstraints(), 16).transpose() << "\n";

    std::cout << "constraint upper first friction block:\n"
              << constraint_upper.segment(mpc.numDynamicsConstraints(), 16).transpose() << "\n";

    std::cout << "solver rows: " << solver_matrix.rows() << ", cols: " << solver_matrix.cols()
              << ", nnz: " << solver_matrix.nonZeros() << "\n";

    const int box0 = mpc.numDynamicsFrictionConstraints();
    std::cout << "first box identity coefficient: " << solver_matrix.coeff(box0, 0) << "\n";

    std::cout << "solver lower first force block:\n"
              << solver_lower.segment(box0 + input0, CentroidalMPC::kInputDim).transpose() << "\n";

    std::cout << "solver upper first force block:\n"
              << solver_upper.segment(box0 + input0, CentroidalMPC::kInputDim).transpose() << "\n";

    std::cout << "solution size: " << solution.size() << "\n";
    std::cout << "solution first state block:\n" << solution.segment(0, CentroidalMPC::kStateDim).transpose() << "\n";

    std::cout << "solution first force block:\n"
              << solution.segment(input0, CentroidalMPC::kInputDim).transpose() << "\n";

    std::cout << "first force matrix rows=[fx fy fz], cols=[FL FR RL RR]:\n" << first_force << "\n";

    std::cout << "solveFirstForce diff norm: " << (solved_first_force - first_force).norm() << "\n";
    return 0;
}