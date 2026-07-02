#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Sparse>
#include <cmath>
#include <iostream>

int main()
{
    Eigen::SparseMatrix<double> hessian(1, 1);
    hessian.insert(0, 0) = 1.0;

    Eigen::VectorXd gradient(1);
    gradient << -1.0;

    Eigen::SparseMatrix<double> linear_matrix(1, 1);
    linear_matrix.insert(0, 0) = 1.0;

    Eigen::VectorXd lower_bound(1);
    lower_bound << 0.0;

    Eigen::VectorXd upper_bound(1);
    upper_bound << 2.0;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(1);
    solver.data()->setNumberOfConstraints(1);

    if (!solver.data()->setHessianMatrix(hessian))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linear_matrix))
        return 1;
    if (!solver.data()->setLowerBound(lower_bound))
        return 1;
    if (!solver.data()->setUpperBound(upper_bound))
        return 1;
    if (!solver.initSolver())
        return 1;

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return 1;

    const double x = solver.getSolution()[0];
    std::cout << "osqp_eigen_smoke solution: " << x << "\n";

    return std::abs(x - 1.0) < 1e-4 ? 0 : 1;
}