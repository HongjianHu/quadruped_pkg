#ifndef QUADRUPED_CONTROLLER__MPC__CENTROIDAL_MPC_H
#define QUADRUPED_CONTROLLER__MPC__CENTROIDAL_MPC_H

#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/mpc/ComTrajectory.h"
#include <Eigen/Sparse>

namespace quadruped_controller
{

class CentroidalMPC
{
  public:
    static constexpr int kStateDim = 12;
    static constexpr int kInputDim = 12;
    static constexpr double kFrictionCoefficient = 0.4;
    static constexpr double kMinNormalForce = 0.0;
    static constexpr double kMaxNormalForce = 180.0;
    static constexpr double kBoundInfinity = 1.0e10;
    explicit CentroidalMPC(int horizon_steps);

    int horizonSteps() const;
    int numVariables() const;
    int numFrictionConstraints() const;
    int stateStartIndex(int knot) const;
    int inputStartIndex(int knot) const;
    int numDynamicsConstraints() const;
    int numDynamicsFrictionConstraints() const;
    int numBoxConstraints() const;
    int numSolverConstraints() const;
    const Eigen::SparseMatrix<double> &hessian() const;
    VecX buildGradient(const ComTrajectory &traj) const;
    const Eigen::SparseMatrix<double> &frictionMatrix() const;
    VecX buildVariableLowerBound(const ComTrajectory &traj) const;
    VecX buildVariableUpperBound(const ComTrajectory &traj) const;
    VecX buildFrictionLowerBound() const;
    VecX buildFrictionUpperBound(const ComTrajectory &traj) const;
    Eigen::SparseMatrix<double> buildDynamicsMatrix(const ComTrajectory &traj) const;
    VecX buildDynamicsBound(const ComTrajectory &traj, const Vec12 &initial_state) const;
    Eigen::SparseMatrix<double> buildConstraintMatrix(const ComTrajectory &traj) const;
    VecX buildConstraintLowerBound(const ComTrajectory &traj, const Vec12 &initial_state) const;
    VecX buildConstraintUpperBound(const ComTrajectory &traj, const Vec12 &initial_state) const;
    Eigen::SparseMatrix<double> buildSolverConstraintMatrix(const ComTrajectory &traj) const;
    VecX buildSolverLowerBound(const ComTrajectory &traj, const Vec12 &initial_state) const;
    VecX buildSolverUpperBound(const ComTrajectory &traj, const Vec12 &initial_state) const;
    VecX solveOnce(const ComTrajectory &traj, const Vec12 &initial_state) const;
    Vec34 extractFirstForce(const VecX &solution) const;
    Vec34 solveFirstForce(const ComTrajectory &traj, const Vec12 &initial_state) const;

  private:
    static Vec12 defaultStateWeights();
    static Vec12 defaultInputWeights();

    void buildHessian();
    void buildFrictionMatrix();
    int horizon_steps_ = 0;
    int num_variables_ = 0;

    Vec12 q_weights_ = Vec12::Zero();
    Vec12 r_weights_ = Vec12::Zero();

    Eigen::SparseMatrix<double> hessian_;
    Eigen::SparseMatrix<double> friction_matrix_;
};

} // namespace quadruped_controller
#endif