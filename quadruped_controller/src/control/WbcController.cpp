#include "quadruped_controller/control/WbcController.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace quadruped_controller
{

WbcController::WbcController()
{
    initializeSparsePatterns();

    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);

    // 对角支撑切换时，未抛光解的激活约束残差可能略高于1e-3。
    // 收紧终止精度并抛光，保持物理残差验收阈值不变。
    solver_.settings()->setAbsoluteTolerance(1.0e-6);
    solver_.settings()->setRelativeTolerance(1.0e-6);
    solver_.settings()->setPolish(true);
    solver_.settings()->setPolishRefineIter(3);
    solver_.settings()->setMaxIteration(4000);

    solver_.data()->setNumberOfVariables(WbcDimensions::kDecisionDof);
    solver_.data()->setNumberOfConstraints(WbcDimensions::kSolverRows);
}

void WbcController::initializeSparsePatterns()
{
    std::vector<Eigen::Triplet<double>> hessian_triplets;
    hessian_triplets.reserve(WbcDimensions::kGeneralizedDof * (WbcDimensions::kGeneralizedDof + 1) / 2 +
                             WbcDimensions::kContactForceDof);

    // 摆动足任务可能使ddq-ddq块变为稠密；OSQP只需要Hessian上三角。
    for (int col = 0; col < WbcDimensions::kGeneralizedDof; ++col)
    {
        for (int row = 0; row <= col; ++row)
        {
            hessian_triplets.emplace_back(row, col, 1.0);
        }
    }

    // 当前接触力目标与正则化只产生力块的对角项。
    for (int force = 0; force < WbcDimensions::kContactForceDof; ++force)
    {
        const int index = WbcDimensions::kForceStart + force;
        hessian_triplets.emplace_back(index, index, 1.0);
    }

    sparse_hessian_.resize(WbcDimensions::kDecisionDof, WbcDimensions::kDecisionDof);
    sparse_hessian_.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());
    sparse_hessian_.makeCompressed();

    std::vector<Eigen::Triplet<double>> constraint_triplets;
    constraint_triplets.reserve(820);

    // 浮动基动力学：M_b*ddq - J_b^T*f = -h_b。
    for (int row = 0; row < WbcDimensions::kFloatingDynamicsRows; ++row)
    {
        for (int col = 0; col < WbcDimensions::kGeneralizedDof; ++col)
        {
            constraint_triplets.emplace_back(row, col, 1.0);
        }
        for (int col = WbcDimensions::kForceStart; col < WbcDimensions::kDecisionDof; ++col)
        {
            constraint_triplets.emplace_back(row, col, 1.0);
        }
    }

    // 同一行块预留支撑腿J*ddq和摆动腿f=0两种结构。
    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        const int row_start = WbcDimensions::kFloatingDynamicsRows + 3 * leg;
        const int force_start = WbcDimensions::kForceStart + 3 * leg;

        for (int local_row = 0; local_row < 3; ++local_row)
        {
            const int row = row_start + local_row;
            for (int col = 0; col < WbcDimensions::kGeneralizedDof; ++col)
            {
                constraint_triplets.emplace_back(row, col, 1.0);
            }
            constraint_triplets.emplace_back(row, force_start + local_row, 1.0);
        }
    }

    const int inequality_row_start = WbcDimensions::kEqualityRows;

    // 每条腿4行摩擦锥和1行法向力界限。
    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        const int row_start = inequality_row_start + WbcDimensions::kFrictionRowsPerLeg * leg;
        const int force_start = WbcDimensions::kForceStart + 3 * leg;

        for (int local_row = 0; local_row < 4; ++local_row)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                constraint_triplets.emplace_back(row_start + local_row, force_start + axis, 1.0);
            }
        }
        constraint_triplets.emplace_back(row_start + 4, force_start + 2, 1.0);
    }

    // 关节力矩界限：M_a*ddq - J_a^T*f。
    const int torque_row_start = inequality_row_start + WbcDimensions::kFrictionRows;
    for (int local_row = 0; local_row < WbcDimensions::kTorqueRows; ++local_row)
    {
        const int row = torque_row_start + local_row;
        for (int col = 0; col < WbcDimensions::kDecisionDof; ++col)
        {
            constraint_triplets.emplace_back(row, col, 1.0);
        }
    }

    sparse_constraint_matrix_.resize(WbcDimensions::kSolverRows, WbcDimensions::kDecisionDof);
    sparse_constraint_matrix_.setFromTriplets(constraint_triplets.begin(), constraint_triplets.end());
    sparse_constraint_matrix_.makeCompressed();
}

void WbcController::updateSparseValues(const WbcObjective &objective, const SolverConstraintMatrix &constraint_matrix)
{
    for (int outer = 0; outer < sparse_hessian_.outerSize(); ++outer)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator entry(sparse_hessian_, outer); entry; ++entry)
        {
            entry.valueRef() = objective.hessian(entry.row(), entry.col());
        }
    }

    for (int outer = 0; outer < sparse_constraint_matrix_.outerSize(); ++outer)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator entry(sparse_constraint_matrix_, outer); entry; ++entry)
        {
            entry.valueRef() = constraint_matrix(entry.row(), entry.col());
        }
    }
}

WbcInput::WbcInput()
{
    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        foot_jacobians_world[leg].setZero();
        foot_jdot_dq_world[leg].setZero();
    }
}

bool WbcController::validateInput(const WbcInput &input, std::string *error) const
{
    const auto fail = [error](const std::string &message) {
        if (error != nullptr)
        {
            *error = message;
        }
        return false;
    };

    if (input.mass_matrix.rows() != WbcDimensions::kGeneralizedDof ||
        input.mass_matrix.cols() != WbcDimensions::kGeneralizedDof)
    {
        return fail("WBC mass matrix must be 18x18");
    }
    if (!input.mass_matrix.allFinite())
    {
        return fail("WBC mass matrix contains non-finite values");
    }
    if (!input.mass_matrix.isApprox(input.mass_matrix.transpose(), 1.0e-8))
    {
        return fail("WBC mass matrix is not symmetric");
    }
    if (!input.nonlinear_effects.allFinite())
    {
        return fail("WBC nonlinear effects contain non-finite values");
    }

    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        if (input.contact[leg] != 0 && input.contact[leg] != 1)
        {
            return fail("WBC contact mask must contain only 0 or 1");
        }
        if (!input.foot_jacobians_world[leg].allFinite())
        {
            return fail("WBC foot Jacobian contains non-finite values");
        }
        if (!input.foot_jdot_dq_world[leg].allFinite())
        {
            return fail("WBC Jdot*dq contains non-finite values");
        }
    }

    if (!input.mpc_contact_forces_world.allFinite() || !input.desired_base_acceleration_tangent.allFinite() ||
        !input.desired_joint_acceleration.allFinite() || !input.desired_swing_foot_acceleration_world.allFinite() ||
        !input.contact_activation.allFinite())
    {
        return fail("WBC reference contains non-finite values");
    }
    if (!input.torque_lower_bound.allFinite() || !input.torque_upper_bound.allFinite())
    {
        return fail("WBC torque bounds contain non-finite values");
    }
    if ((input.torque_lower_bound.array() > input.torque_upper_bound.array()).any())
    {
        return fail("WBC torque lower bound exceeds upper bound");
    }
    if (!std::isfinite(input.friction_coefficient) || !input.min_normal_force.allFinite() ||
        !input.max_normal_force.allFinite())
    {
        return fail("WBC contact limits contain non-finite values");
    }

    if (input.friction_coefficient <= 0.0)
    {
        return fail("WBC friction coefficient must be positive");
    }

    if ((input.contact_activation.array() < 0.0).any() || (input.contact_activation.array() > 1.0).any())
    {
        return fail("WBC contact activation must be in [0, 1]");
    }

    if ((input.min_normal_force.array() < 0.0).any() ||
        (input.max_normal_force.array() < input.min_normal_force.array()).any())
    {
        return fail("WBC normal force bounds are invalid");
    }

    if (error != nullptr)
    {
        error->clear();
    }
    return true;
}

WbcEqualitySystem WbcController::buildEqualitySystem(const WbcInput &input) const
{
    std::string error;
    if (!validateInput(input, &error))
    {
        throw std::invalid_argument(error);
    }

    WbcEqualitySystem system;

    constexpr int base_rows = WbcDimensions::kFloatingDynamicsRows;

    system.matrix.block<base_rows, WbcDimensions::kGeneralizedDof>(0, WbcDimensions::kAccelerationStart) =
        input.mass_matrix.topRows<base_rows>();

    /*
     * 浮动基动力学：
     *
     * M_b * ddq - J_b^T * f = -h_b
     */

    system.bound.head<base_rows>() = -input.nonlinear_effects.head<base_rows>();

    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        const int force_col = WbcDimensions::kForceStart + 3 * leg;

        system.matrix.block<base_rows, 3>(0, force_col) =
            -input.foot_jacobians_world[leg].leftCols<base_rows>().transpose();
    }

    /*
     * 后12行根据接触模式切换：
     *
     * 支撑腿：J_i * ddq = -Jdot_i * dq
     * 摆动腿：f_i = 0
     */

    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        const int row = WbcDimensions::kFloatingBaseDof + 3 * leg;

        const int force_col = WbcDimensions::kForceStart + 3 * leg;

        if (input.contact[leg] == 1)
        {
            system.matrix.block<3, WbcDimensions::kGeneralizedDof>(row, WbcDimensions::kAccelerationStart) =
                input.foot_jacobians_world[leg];

            system.bound.segment<3>(row) = -input.foot_jdot_dq_world[leg];
        }
        else
        {
            system.matrix.block<3, 3>(row, force_col) = Mat3::Identity();
            system.bound.segment<3>(row).setZero();
        }
    }

    return system;
}

WbcObjective WbcController::buildObjective(const WbcInput &input, const WbcWeights &weights) const
{
    std::string error;
    if (!validateInput(input, &error))
    {
        throw std::invalid_argument(error);
    }

    const std::array<double, 6> weight_values = {weights.base_acceleration,           weights.swing_foot_acceleration,
                                                 weights.joint_acceleration,          weights.contact_force_tracking,
                                                 weights.acceleration_regularization, weights.force_regularization};

    for (const double weight : weight_values)
    {
        if (!std::isfinite(weight) || weight < 0.0)
        {
            throw std::invalid_argument("WBC weights must be finite and non-negative");
        }
    }

    if (weights.acceleration_regularization <= 0.0 || weights.force_regularization <= 0.0)
    {
        throw std::invalid_argument("WBC regularization weights must be positive");
    }

    WbcObjective objective;

    constexpr int ddq_start = WbcDimensions::kAccelerationStart;

    constexpr int force_start = WbcDimensions::kForceStart;

    /*
     * 正则化：
     *
     * w_ddq * ||ddq||²
     * w_force * ||f||²
     */
    objective.hessian.block<WbcDimensions::kGeneralizedDof, WbcDimensions::kGeneralizedDof>(ddq_start, ddq_start)
        .diagonal()
        .array() += 2.0 * weights.acceleration_regularization;

    objective.hessian.block<WbcDimensions::kContactForceDof, WbcDimensions::kContactForceDof>(force_start, force_start)
        .diagonal()
        .array() += 2.0 * weights.force_regularization;
    /*
     * 基座加速度：
     *
     * w_base * ||ddq_base - ddq_base_des||²
     */
    objective.hessian.block<6, 6>(ddq_start, ddq_start).diagonal().array() += 2.0 * weights.base_acceleration;
    objective.gradient.head<6>() -= 2.0 * weights.base_acceleration * input.desired_base_acceleration_tangent;
    /*
     * 关节加速度：
     *
     * w_joint * ||ddq_joint - ddq_joint_des||²
     */
    constexpr int joint_ddq_start = WbcDimensions::kFloatingBaseDof;
    objective.hessian.block<12, 12>(joint_ddq_start, joint_ddq_start).diagonal().array() +=
        2.0 * weights.joint_acceleration;
    objective.gradient.segment<12>(joint_ddq_start) -=
        2.0 * weights.joint_acceleration * input.desired_joint_acceleration;
    /*
     * MPC接触力跟踪：
     *
     * w_force_track * ||f - f_mpc||²
     */
    Vec34 force_reference = input.mpc_contact_forces_world;
    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        if (input.contact[leg] == 0)
        {
            force_reference.col(leg).setZero();
        }
        else
        {
            force_reference.col(leg) *= input.contact_activation[leg];
        }
    }
    const Vec12 force_reference_vector = vec34ToVec12(force_reference);

    objective.hessian.block<12, 12>(force_start, force_start).diagonal().array() +=
        2.0 * weights.contact_force_tracking;

    objective.gradient.segment<12>(force_start) -= 2.0 * weights.contact_force_tracking * force_reference_vector;
    /*
     * 摆动足加速度（低于基座任务的关节子空间任务）：
     *
     * J_b*ddq_b + J_j*ddq_j + Jdot*dq ≈ a_des
     *
     * 基座加速度由高优先级任务给定，因此将其期望值移到右端，只优化关节加速度：
     *
     * J_j*ddq_j ≈ a_des - Jdot*dq - J_b*ddq_b_des
     *
     * 这样摆动足误差不会直接改写Hessian左上角的基座六维变量。若仍使用完整J，
     * 大幅的触地搜索加速度会通过J_b与机身任务竞争，造成竖直加速度周期性偏离。
     */
    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        if (input.contact[leg] == 1)
        {
            continue;
        }

        const WbcFootJacobian &jacobian = input.foot_jacobians_world[leg];
        const auto joint_jacobian = jacobian.rightCols<WbcDimensions::kActuatedDof>();

        const Vec3 acceleration_target =
            input.desired_swing_foot_acceleration_world.col(leg) - input.foot_jdot_dq_world[leg] -
            jacobian.leftCols<WbcDimensions::kFloatingBaseDof>() * input.desired_base_acceleration_tangent;

        objective.hessian.block<WbcDimensions::kActuatedDof, WbcDimensions::kActuatedDof>(
            joint_ddq_start, joint_ddq_start) +=
            2.0 * weights.swing_foot_acceleration * joint_jacobian.transpose() * joint_jacobian;

        objective.gradient.segment<WbcDimensions::kActuatedDof>(joint_ddq_start) -=
            2.0 * weights.swing_foot_acceleration * joint_jacobian.transpose() * acceleration_target;
    }

    // 消除数值计算造成的微小非对称
    objective.hessian = 0.5 * (objective.hessian + objective.hessian.transpose());

    return objective;
}

WbcInequalitySystem WbcController::buildInequalitySystem(const WbcInput &input) const
{
    std::string error;
    if (!validateInput(input, &error))
    {
        throw std::invalid_argument(error);
    }

    WbcInequalitySystem system;

    const double mu = input.friction_coefficient;

    /*
     * 每条腿5行：
     *
     *  fx + fy - mu*fz <= 0
     *  fx - fy - mu*fz <= 0
     * -fx + fy - mu*fz <= 0
     * -fx - fy - mu*fz <= 0
     *  fz_min <= fz <= fz_max
     */

    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        const int row = WbcDimensions::kFrictionRowsPerLeg * leg;

        const int force_col = WbcDimensions::kForceStart + 3 * leg;

        const int fx = force_col;
        const int fy = force_col + 1;
        const int fz = force_col + 2;

        system.matrix(row + 0, fx) = 1.0;
        system.matrix(row + 0, fy) = 1.0;
        system.matrix(row + 0, fz) = -mu;

        system.matrix(row + 1, fx) = 1.0;
        system.matrix(row + 1, fy) = -1.0;
        system.matrix(row + 1, fz) = -mu;

        system.matrix(row + 2, fx) = -1.0;
        system.matrix(row + 2, fy) = 1.0;
        system.matrix(row + 2, fz) = -mu;

        system.matrix(row + 3, fx) = -1.0;
        system.matrix(row + 3, fy) = -1.0;
        system.matrix(row + 3, fz) = -mu;

        system.upper.segment<4>(row).setZero();

        const int normal_row = row + 4;
        system.matrix(normal_row, fz) = 1.0;

        if (input.contact[leg] == 1)
        {
            system.lower[normal_row] = input.min_normal_force[leg];

            system.upper[normal_row] = input.max_normal_force[leg];
        }
        else
        {
            system.lower[normal_row] = 0.0;

            system.upper[normal_row] = 0.0;
        }
    }

    /*
     * tau = M_a*ddq + h_a - J_a^T*f
     *
     * tau_lower <= tau <= tau_upper
     *
     * 移动h_a：
     *
     * tau_lower - h_a
     * <=
     * M_a*ddq - J_a^T*f
     * <=
     * tau_upper - h_a
     */
    constexpr int torque_row = WbcDimensions::kFrictionRows;

    system.matrix.block<12, 18>(torque_row, WbcDimensions::kAccelerationStart) = input.mass_matrix.bottomRows<12>();

    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        const int force_col = WbcDimensions::kForceStart + 3 * leg;

        system.matrix.block<12, 3>(torque_row, force_col) = -input.foot_jacobians_world[leg].rightCols(12).transpose();
    }

    const Vec12 actuated_nonlinear_effects = input.nonlinear_effects.tail<12>();

    system.lower.segment<12>(torque_row) = input.torque_lower_bound - actuated_nonlinear_effects;

    system.upper.segment<12>(torque_row) = input.torque_upper_bound - actuated_nonlinear_effects;

    return system;
}

WbcOutput WbcController::solve(const WbcInput &input, const WbcWeights &weights)
{
    WbcOutput output;
    try
    {
        std::string input_error;
        if (!validateInput(input, &input_error))
        {
            output.status = input_error;
            return output;
        }

        const bool contact_changed =
            has_previous_contact_ && (input.contact.array() != previous_contact_.array()).any();

        const WbcObjective objective = buildObjective(input, weights);

        const WbcEqualitySystem equality = buildEqualitySystem(input);

        const WbcInequalitySystem inequality = buildInequalitySystem(input);

        SolverConstraintMatrix dense_constraint_matrix = SolverConstraintMatrix::Zero();

        dense_constraint_matrix.topRows<WbcDimensions::kEqualityRows>() = equality.matrix;

        dense_constraint_matrix.bottomRows<WbcDimensions::kInequalityRows>() = inequality.matrix;

        solver_lower_bound_.head<WbcDimensions::kEqualityRows>() = equality.bound;
        solver_upper_bound_.head<WbcDimensions::kEqualityRows>() = equality.bound;
        solver_lower_bound_.tail<WbcDimensions::kInequalityRows>() = inequality.lower;
        solver_upper_bound_.tail<WbcDimensions::kInequalityRows>() = inequality.upper;
        solver_gradient_ = objective.gradient;

        updateSparseValues(objective, dense_constraint_matrix);

        if (!solver_initialized_)
        {
            if (!solver_.data()->setHessianMatrix(sparse_hessian_) || !solver_.data()->setGradient(solver_gradient_) ||
                !solver_.data()->setLinearConstraintsMatrix(sparse_constraint_matrix_) ||
                !solver_.data()->setLowerBound(solver_lower_bound_) ||
                !solver_.data()->setUpperBound(solver_upper_bound_))
            {
                output.status = "WBC failed to set initial OSQP data";
                return output;
            }

            if (!solver_.initSolver())
            {
                output.status = "WBC failed to initialize OSQP";
                return output;
            }

            solver_initialized_ = true;
            ++solver_initialization_count_;
        }
        else
        {
            /*
             * 稀疏索引已经固定，因此一次更新P和A的全部数值数组。
             * nullptr索引表示按原CSC顺序替换全部数值，不改变非零结构。
             */
            const OSQPInt matrix_update_flag = osqp_update_data_mat(
                solver_.solver().get(), sparse_hessian_.valuePtr(), nullptr,
                static_cast<OSQPInt>(sparse_hessian_.nonZeros()), sparse_constraint_matrix_.valuePtr(), nullptr,
                static_cast<OSQPInt>(sparse_constraint_matrix_.nonZeros()));

            if (matrix_update_flag != 0)
            {
                output.status = "WBC failed to update Hessian/constraint values";
                return output;
            }
            if (!solver_.updateGradient(solver_gradient_))
            {
                output.status = "WBC failed to update gradient";
                return output;
            }
            if (!solver_.updateBounds(solver_lower_bound_, solver_upper_bound_))
            {
                output.status = "WBC failed to update bounds";
                return output;
            }
        }

        /*
         * 连续控制周期保留warm start；接触集合发生离散切换时，旧对偶变量
         * 对应的是上一组接触约束，清零变量可避免错误的不可行判定。
         * solver workspace和稀疏结构都不重建。
         */
        if (contact_changed && !solver_.clearSolverVariables())
        {
            output.status = "WBC failed to reset warm start after contact change";
            return output;
        }

        previous_contact_ = input.contact;
        has_previous_contact_ = true;

        const OsqpEigen::ErrorExitFlag solve_flag = solver_.solveProblem();

        if (solve_flag != OsqpEigen::ErrorExitFlag::NoError)
        {
            output.status = "WBC OSQP solve failed";
            return output;
        }

        const OsqpEigen::Status solver_status = solver_.getStatus();
        if (solver_status != OsqpEigen::Status::Solved && solver_status != OsqpEigen::Status::SolvedInaccurate)
        {
            solver_.clearSolverVariables();
            output.status = "WBC OSQP status is not solved: " + std::to_string(static_cast<int>(solver_status));
            return output;
        }

        const VecX solution = solver_.getSolution();

        if (solution.size() != WbcDimensions::kDecisionDof)
        {
            output.status = "WBC solution size mismatch";
            return output;
        }

        if (!solution.allFinite())
        {
            output.status = "WBC solution contains non-finite values";
            return output;
        }

        /*
         * z = [ddq(18), f(12)]
         */
        output.generalized_acceleration =
            solution.segment<WbcDimensions::kGeneralizedDof>(WbcDimensions::kAccelerationStart);

        for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
        {
            const int force_start = WbcDimensions::kForceStart + 3 * leg;

            output.contact_forces_world.col(leg) = solution.segment<3>(force_start);
        }

        output.joint_torques = recoverJointTorques(input, output.generalized_acceleration, output.contact_forces_world);

        output.dynamics_residual_norm = computeDynamicsResidual(input, output).norm();

        /*
         * 检查全部18行等式。
         */
        const WbcEqualityVector equality_residual = equality.matrix * solution - equality.bound;

        output.equality_residual_norm = equality_residual.norm();

        /*
         * 检查32行不等式。
         */
        const WbcInequalityVector inequality_value = inequality.matrix * solution;

        const double lower_violation = (inequality.lower - inequality_value).maxCoeff();

        const double upper_violation = (inequality_value - inequality.upper).maxCoeff();

        output.max_inequality_violation = std::max({0.0, lower_violation, upper_violation});

        /*
         * OSQP返回成功后仍然进行物理残差检查。
         */
        constexpr double kSolutionTolerance = 1.0e-3;

        if (output.dynamics_residual_norm > kSolutionTolerance || output.equality_residual_norm > kSolutionTolerance ||
            output.max_inequality_violation > kSolutionTolerance)
        {
            output.status = "WBC solution validation failed";
            return output;
        }

        output.success = true;
        output.status = "solved";
        return output;
    }
    catch (const std::exception &exception)
    {
        output.status = std::string("WBC exception: ") + exception.what();

        return output;
    }
}

Vec18 WbcController::computeDynamicsResidual(const WbcInput &input, const WbcOutput &output) const
{
    std::string error;
    if (!validateInput(input, &error))
    {
        throw std::invalid_argument(error);
    }
    if (!output.generalized_acceleration.allFinite() || !output.contact_forces_world.allFinite() ||
        !output.joint_torques.allFinite())
    {
        throw std::invalid_argument("WBC output contains non-finite values");
    }

    Vec18 generalized_actuation = Vec18::Zero();
    generalized_actuation.tail<WbcDimensions::kActuatedDof>() = output.joint_torques;

    Vec18 generalized_contact_force = Vec18::Zero();
    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        generalized_contact_force.noalias() +=
            input.foot_jacobians_world[leg].transpose() * output.contact_forces_world.col(leg);
    }

    return input.mass_matrix * output.generalized_acceleration + input.nonlinear_effects - generalized_actuation -
           generalized_contact_force;
}

Vec12 WbcController::recoverJointTorques(const WbcInput &input, const Vec18 &generalized_acceleration,
                                         const Vec34 &contact_forces_world) const
{
    std::string error;
    if (!validateInput(input, &error))
    {
        throw std::invalid_argument(error);
    }

    if (!generalized_acceleration.allFinite() || !contact_forces_world.allFinite())
    {
        throw std::invalid_argument("WBC solution contains non-finite values");
    }

    /*
     * 驱动关节动力学：
     *
     * M_a * ddq + h_a = tau + J_a^T * f
     *
     * 因此：
     *
     * tau = M_a * ddq + h_a - J_a^T * f
     */

    Vec12 joint_torques = input.mass_matrix.bottomRows<WbcDimensions::kActuatedDof>() * generalized_acceleration;
    joint_torques += input.nonlinear_effects.tail<WbcDimensions::kActuatedDof>();

    for (int leg = 0; leg < WbcDimensions::kNumLegs; ++leg)
    {
        joint_torques.noalias() -=
            input.foot_jacobians_world[leg].rightCols<WbcDimensions::kActuatedDof>().transpose() *
            contact_forces_world.col(leg);
    }

    return joint_torques;
}
} // namespace quadruped_controller
