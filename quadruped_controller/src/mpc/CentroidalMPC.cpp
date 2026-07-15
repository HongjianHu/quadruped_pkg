#include "quadruped_controller/mpc/CentroidalMPC.h"

#include <OsqpEigen/OsqpEigen.h>
#include <stdexcept>
#include <vector>
namespace quadruped_controller
{

CentroidalMPC::CentroidalMPC(int horizon_steps)
    : horizon_steps_(horizon_steps), num_variables_(horizon_steps * (kStateDim + kInputDim)),
      q_weights_(defaultStateWeights()), r_weights_(defaultInputWeights())
{
    if (horizon_steps_ <= 0)
    {
        throw std::runtime_error("CentroidalMPC horizon_steps must be positive");
    }

    buildHessian();
    buildFrictionMatrix();
}

Vec12 CentroidalMPC::defaultStateWeights()
{
    Vec12 weights;
    weights << 1.0, 1.0, 50.0, 10.0, 20.0, 1.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0;
    return weights;
}

Vec12 CentroidalMPC::defaultInputWeights()
{
    Vec12 weights;
    weights.setConstant(1e-5);
    return weights;
}

int CentroidalMPC::horizonSteps() const
{
    return horizon_steps_;
}

int CentroidalMPC::numVariables() const
{
    return num_variables_;
}

int CentroidalMPC::numDynamicsConstraints() const
{
    return horizon_steps_ * kStateDim;
}

int CentroidalMPC::numFrictionConstraints() const
{
    return 4 * 4 * horizon_steps_;
}

int CentroidalMPC::numDynamicsFrictionConstraints() const
{
    return numDynamicsConstraints() + numFrictionConstraints();
}

int CentroidalMPC::numBoxConstraints() const
{
    return num_variables_;
}

int CentroidalMPC::numSolverConstraints() const
{
    return numDynamicsFrictionConstraints() + numBoxConstraints();
}

int CentroidalMPC::stateStartIndex(int knot) const
{
    return knot * kStateDim;
}

int CentroidalMPC::inputStartIndex(int knot) const
{
    return horizon_steps_ * kStateDim + knot * kInputDim;
}

const Eigen::SparseMatrix<double> &CentroidalMPC::hessian() const
{
    return hessian_;
}

const Eigen::SparseMatrix<double> &CentroidalMPC::frictionMatrix() const
{
    return friction_matrix_;
}

void CentroidalMPC::buildHessian()
{
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(horizon_steps_ * (kStateDim + kInputDim)));

    for (int k = 0; k < horizon_steps_; ++k)
    {
        const int state_base = stateStartIndex(k);
        for (int i = 0; i < kStateDim; ++i)
        {
            if (q_weights_[i] != 0.0)
            {
                triplets.emplace_back(state_base + i, state_base + i, 2.0 * q_weights_[i]);
            }
        }

        const int input_base = inputStartIndex(k);
        for (int i = 0; i < kInputDim; ++i)
        {
            if (r_weights_[i] != 0.0)
            {
                triplets.emplace_back(input_base + i, input_base + i, 2.0 * r_weights_[i]);
            }
        }
    }
    hessian_.resize(num_variables_, num_variables_);
    hessian_.setFromTriplets(triplets.begin(), triplets.end());
    hessian_.makeCompressed();
}

VecX CentroidalMPC::buildGradient(const ComTrajectory &traj) const
{
    if (traj.horizonSteps() != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC trajectory horizon does not match MPC horizon");
    }

    const MatX x_ref = traj.computeXRefVec();

    if (x_ref.rows() != kStateDim || x_ref.cols() != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC x_ref shape mismatch");
    }

    VecX gradient = VecX::Zero(num_variables_);

    for (int k = 0; k < horizon_steps_; ++k)
    {
        const int state_base = stateStartIndex(k);
        for (int i = 0; i < kStateDim; ++i)
        {
            gradient[state_base + i] = -2.0 * q_weights_[i] * x_ref(i, k);
        }
    }

    return gradient;
}

void CentroidalMPC::buildFrictionMatrix()
{
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(numFrictionConstraints() * 2));

    int row = 0;
    for (int k = 0; k < horizon_steps_; ++k)
    {
        const int input_base = inputStartIndex(k);
        for (int leg = 0; leg < 4; ++leg)
        {
            const int fx = input_base + 3 * leg;
            const int fy = input_base + 3 * leg + 1;
            const int fz = input_base + 3 * leg + 2;

            // fx + fy - mu * fz <= 0
            triplets.emplace_back(row, fx, 1.0);
            triplets.emplace_back(row, fy, 1.0);
            triplets.emplace_back(row, fz, -kFrictionCoefficient);
            ++row;

            // fx - fy - mu * fz <= 0
            triplets.emplace_back(row, fx, 1.0);
            triplets.emplace_back(row, fy, -1.0);
            triplets.emplace_back(row, fz, -kFrictionCoefficient);
            ++row;

            // -fx + fy - mu * fz <= 0
            triplets.emplace_back(row, fx, -1.0);
            triplets.emplace_back(row, fy, 1.0);
            triplets.emplace_back(row, fz, -kFrictionCoefficient);
            ++row;

            // -fx - fy - mu * fz <= 0
            triplets.emplace_back(row, fx, -1.0);
            triplets.emplace_back(row, fy, -1.0);
            triplets.emplace_back(row, fz, -kFrictionCoefficient);
            ++row;
        }
    }
    friction_matrix_.resize(numFrictionConstraints(), num_variables_);
    friction_matrix_.setFromTriplets(triplets.begin(), triplets.end());
    friction_matrix_.makeCompressed();
}

VecX CentroidalMPC::buildVariableLowerBound(const ComTrajectory &traj) const
{
    if (traj.horizonSteps() != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC trajectory horizon does not match MPC horizon");
    }
    VecX lower_bound = VecX::Constant(num_variables_, -kBoundInfinity);
    const auto &contact_table = traj.contactTable();

    for (int k = 0; k < horizon_steps_; ++k)
    {
        const int input_base = inputStartIndex(k);
        for (int leg = 0; leg < 4; ++leg)
        {
            const int force_base = input_base + 3 * leg;

            if (contact_table(leg, k) == 0)
            {
                lower_bound.segment<3>(force_base).setZero();
            }
            else
            {
                lower_bound[force_base + 2] = kMinNormalForce;
            }
        }
    }
    return lower_bound;
}

VecX CentroidalMPC::buildVariableUpperBound(const ComTrajectory &traj) const
{
    if (traj.horizonSteps() != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC trajectory horizon does not match MPC horizon");
    }
    VecX upper_bound = VecX::Constant(num_variables_, kBoundInfinity);
    const auto &contact_table = traj.contactTable();

    for (int k = 0; k < horizon_steps_; ++k)
    {
        const int input_base = inputStartIndex(k);

        for (int leg = 0; leg < 4; ++leg)
        {
            const int force_base = input_base + 3 * leg;

            if (contact_table(leg, k) == 0)
            {
                upper_bound.segment<3>(force_base).setZero();
            }
            else
            {
                upper_bound[force_base + 2] = CentroidalMPC::kMaxNormalForce;
            }
        }
    }

    return upper_bound;
}

VecX CentroidalMPC::buildFrictionLowerBound() const
{
    return VecX::Constant(numFrictionConstraints(), -kBoundInfinity);
}

VecX CentroidalMPC::buildFrictionUpperBound(const ComTrajectory &traj) const
{
    if (traj.horizonSteps() != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC trajectory horizon does not match MPC horizon");
    }

    VecX upper_bound = VecX::Constant(numFrictionConstraints(), kBoundInfinity);
    const auto &contact_table = traj.contactTable();

    for (int k = 0; k < horizon_steps_; ++k)
    {
        for (int leg = 0; leg < 4; ++leg)
        {
            const int row_base = 16 * k + 4 * leg;

            if (contact_table(leg, k) == 1)
            {
                upper_bound.segment<4>(row_base).setZero();
            }
        }
    }

    return upper_bound;
}

Eigen::SparseMatrix<double> CentroidalMPC::buildDynamicsMatrix(const ComTrajectory &traj) const
{
    if (traj.horizonSteps() != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC trajectory horizon does not match MPC horizon");
    }

    const auto &bd = traj.discreteB();

    if (static_cast<int>(bd.size()) != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC discreteB horizon mismatch");
    }

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(
        static_cast<std::size_t>(horizon_steps_ * (kStateDim + kStateDim * kStateDim + kStateDim * kInputDim)));
    const Mat12 &ad = traj.discreteA();

    for (int k = 0; k < horizon_steps_; ++k)
    {
        const int row_base = k * kStateDim;
        const int state_base = stateStartIndex(k);
        const int input_base = inputStartIndex(k);

        for (int row = 0; row < kStateDim; ++row)
        {
            triplets.emplace_back(row_base + row, state_base + row, 1.0);
        }

        if (k > 0)
        {
            const int prev_state_base = stateStartIndex(k - 1);
            for (int row = 0; row < kStateDim; ++row)
            {
                for (int col = 0; col < kStateDim; ++col)
                {
                    const double value = -ad(row, col);
                    if (value != 0.0)
                    {
                        triplets.emplace_back(row_base + row, prev_state_base + col, value);
                    }
                }
            }
        }

        for (int row = 0; row < kStateDim; ++row)
        {
            for (int col = 0; col < kInputDim; ++col)
            {
                const double value = -bd[static_cast<std::size_t>(k)](row, col);
                if (value != 0.0)
                {
                    triplets.emplace_back(row_base + row, input_base + col, value);
                }
            }
        }
    }
    Eigen::SparseMatrix<double> dynamics_matrix(numDynamicsConstraints(), numVariables());
    dynamics_matrix.setFromTriplets(triplets.begin(), triplets.end());
    dynamics_matrix.makeCompressed();
    return dynamics_matrix;
}

VecX CentroidalMPC::buildDynamicsBound(const ComTrajectory &traj, const Vec12 &initial_state) const
{
    if (traj.horizonSteps() != horizon_steps_)
    {
        throw std::runtime_error("CentroidalMPC trajectory horizon does not match MPC horizon");
    }

    VecX bound = VecX::Zero(numDynamicsConstraints());

    const Mat12 &ad = traj.discreteA();
    const Vec12 &gd = traj.discreteGravity();

    bound.segment<kStateDim>(0) = ad * initial_state + gd;

    for (int k = 1; k < horizon_steps_; ++k)
    {
        bound.segment<kStateDim>(k * kStateDim) = gd;
    }

    return bound;
}

Eigen::SparseMatrix<double> CentroidalMPC::buildConstraintMatrix(const ComTrajectory &traj) const
{
    const Eigen::SparseMatrix<double> dynamics_matrix = buildDynamicsMatrix(traj);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(dynamics_matrix.nonZeros() + friction_matrix_.nonZeros()));

    for (int outer = 0; outer < dynamics_matrix.outerSize(); ++outer)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(dynamics_matrix, outer); it; ++it)
        {
            triplets.emplace_back(it.row(), it.col(), it.value());
        }
    }

    const int friction_row_offset = numDynamicsConstraints();
    for (int outer = 0; outer < friction_matrix_.outerSize(); ++outer)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(friction_matrix_, outer); it; ++it)
        {
            triplets.emplace_back(friction_row_offset + it.row(), it.col(), it.value());
        }
    }

    Eigen::SparseMatrix<double> constraint_matrix(numDynamicsFrictionConstraints(), numVariables());
    constraint_matrix.setFromTriplets(triplets.begin(), triplets.end());
    constraint_matrix.makeCompressed();

    return constraint_matrix;
}

VecX CentroidalMPC::buildConstraintLowerBound(const ComTrajectory &traj, const Vec12 &initial_state) const
{
    VecX lower_bound = VecX::Zero(numDynamicsFrictionConstraints());

    lower_bound.segment(0, numDynamicsConstraints()) = buildDynamicsBound(traj, initial_state);
    lower_bound.segment(numDynamicsConstraints(), numFrictionConstraints()) = buildFrictionLowerBound();

    return lower_bound;
}

VecX CentroidalMPC::buildConstraintUpperBound(const ComTrajectory &traj, const Vec12 &initial_state) const
{
    VecX upper_bound = VecX::Zero(numDynamicsFrictionConstraints());

    upper_bound.segment(0, numDynamicsConstraints()) = buildDynamicsBound(traj, initial_state);
    upper_bound.segment(numDynamicsConstraints(), numFrictionConstraints()) = buildFrictionUpperBound(traj);

    return upper_bound;
}

Eigen::SparseMatrix<double> CentroidalMPC::buildSolverConstraintMatrix(const ComTrajectory &traj) const
{
    const Eigen::SparseMatrix<double> constraint_matrix = buildConstraintMatrix(traj);

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(constraint_matrix.nonZeros() + numVariables()));

    for (int outer = 0; outer < constraint_matrix.outerSize(); ++outer)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(constraint_matrix, outer); it; ++it)
        {
            triplets.emplace_back(it.row(), it.col(), it.value());
        }
    }
    const int box_row_offset = numDynamicsFrictionConstraints();
    for (int i = 0; i < numVariables(); ++i)
    {
        triplets.emplace_back(box_row_offset + i, i, 1.0);
    }

    Eigen::SparseMatrix<double> solver_matrix(numSolverConstraints(), numVariables());
    solver_matrix.setFromTriplets(triplets.begin(), triplets.end());
    solver_matrix.makeCompressed();

    return solver_matrix;
}

VecX CentroidalMPC::buildSolverLowerBound(const ComTrajectory &traj, const Vec12 &initial_state) const
{
    VecX lower_bound = VecX::Zero(numSolverConstraints());

    lower_bound.segment(0, numDynamicsFrictionConstraints()) = buildConstraintLowerBound(traj, initial_state);

    lower_bound.segment(numDynamicsFrictionConstraints(), numBoxConstraints()) = buildVariableLowerBound(traj);

    return lower_bound;
}

VecX CentroidalMPC::buildSolverUpperBound(const ComTrajectory &traj, const Vec12 &initial_state) const
{
    VecX upper_bound = VecX::Zero(numSolverConstraints());

    upper_bound.segment(0, numDynamicsFrictionConstraints()) = buildConstraintUpperBound(traj, initial_state);

    upper_bound.segment(numDynamicsFrictionConstraints(), numBoxConstraints()) = buildVariableUpperBound(traj);

    return upper_bound;
}

VecX CentroidalMPC::solveOnce(const ComTrajectory &traj, const Vec12 &initial_state) const
{
    Eigen::SparseMatrix<double> hessian_copy = hessian_;
    Eigen::SparseMatrix<double> solver_matrix = buildSolverConstraintMatrix(traj);

    VecX gradient = buildGradient(traj);
    VecX lower_bound = buildSolverLowerBound(traj, initial_state);
    VecX upper_bound = buildSolverUpperBound(traj, initial_state);

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);
    solver.settings()->setMaxIteration(1000);

    solver.data()->setNumberOfVariables(numVariables());
    solver.data()->setNumberOfConstraints(numSolverConstraints());

    if (!solver.data()->setHessianMatrix(hessian_copy))
    {
        throw std::runtime_error("CentroidalMPC failed to set Hessian matrix");
    }
    if (!solver.data()->setGradient(gradient))
    {
        throw std::runtime_error("CentroidalMPC failed to set gradient");
    }
    if (!solver.data()->setLinearConstraintsMatrix(solver_matrix))
    {
        throw std::runtime_error("CentroidalMPC failed to set constraint matrix");
    }
    if (!solver.data()->setLowerBound(lower_bound))
    {
        throw std::runtime_error("CentroidalMPC failed to set lower bound");
    }
    if (!solver.data()->setUpperBound(upper_bound))
    {
        throw std::runtime_error("CentroidalMPC failed to set upper bound");
    }
    if (!solver.initSolver())
    {
        throw std::runtime_error("CentroidalMPC failed to initialize OSQP solver");
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        throw std::runtime_error("CentroidalMPC OSQP solve failed");
    }

    return solver.getSolution();
}

Vec34 CentroidalMPC::extractFirstForce(const VecX &solution) const
{
    if (solution.size() != numVariables())
    {
        throw std::runtime_error("CentroidalMPC solution has unexpected size.");
    }

    Vec34 forces = Vec34::Zero();
    const int input0 = inputStartIndex(0);

    for (int leg = 0; leg < 4; ++leg)
    {
        forces.col(leg) = solution.segment<3>(input0 + 3 * leg);
    }

    return forces;
}

Vec34 CentroidalMPC::solveFirstForce(const ComTrajectory &traj, const Vec12 &initial_state) const
{
    const VecX solution = solveOnce(traj, initial_state);
    return extractFirstForce(solution);
}
} // namespace quadruped_controller
