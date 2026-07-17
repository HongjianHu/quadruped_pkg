#ifndef QUADRUPED_CONTROLLER__CONTROL__WBC_CONTROLLER_H
#define QUADRUPED_CONTROLLER__CONTROL__WBC_CONTROLLER_H

#include "quadruped_controller/common/mathTypes.h"

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Sparse>

#include <array>
#include <cstddef>
#include <limits>
#include <string>

namespace quadruped_controller
{

struct WbcDimensions
{
    static constexpr int kNumLegs = 4;
    static constexpr int kFloatingBaseDof = 6;
    static constexpr int kActuatedDof = 12;
    static constexpr int kGeneralizedDof = kFloatingBaseDof + kActuatedDof;
    static constexpr int kContactForceDof = 3 * kNumLegs;
    static constexpr int kDecisionDof = kGeneralizedDof + kContactForceDof;

    static constexpr int kAccelerationStart = 0;
    static constexpr int kForceStart = kGeneralizedDof;

    static constexpr int kFloatingDynamicsRows = kFloatingBaseDof;

    static constexpr int kContactModeRows = 3 * kNumLegs;

    static constexpr int kEqualityRows = kFloatingDynamicsRows + kContactModeRows;

    // 每条腿4行摩擦锥, 每条腿1行法向力上下限；
    static constexpr int kFrictionRowsPerLeg = 5;
    static constexpr int kFrictionRows = kFrictionRowsPerLeg * kNumLegs;
    static constexpr int kTorqueRows = kActuatedDof;
    static constexpr int kInequalityRows = kFrictionRows + kTorqueRows;

    static constexpr int kSolverRows = kEqualityRows + kInequalityRows;
};

using WbcFootJacobian = Eigen::Matrix<double, 3, WbcDimensions::kGeneralizedDof>;
using WbcEqualityMatrix = Eigen::Matrix<double, WbcDimensions::kEqualityRows, WbcDimensions::kDecisionDof>;
using WbcEqualityVector = Eigen::Matrix<double, WbcDimensions::kEqualityRows, 1>;
using WbcHessian = Eigen::Matrix<double, WbcDimensions::kDecisionDof, WbcDimensions::kDecisionDof>;
using WbcGradient = Eigen::Matrix<double, WbcDimensions::kDecisionDof, 1>;
using WbcInequalityMatrix = Eigen::Matrix<double, WbcDimensions::kInequalityRows, WbcDimensions::kDecisionDof>;
using WbcInequalityVector = Eigen::Matrix<double, WbcDimensions::kInequalityRows, 1>;

struct WbcEqualitySystem
{
    WbcEqualityMatrix matrix = WbcEqualityMatrix::Zero();
    WbcEqualityVector bound = WbcEqualityVector::Zero();
};

struct WbcWeights
{
    double base_acceleration = 100.0;
    double swing_foot_acceleration = 100.0;
    double joint_acceleration = 1.0;
    double contact_force_tracking = 0.01;

    double acceleration_regularization = 1.0e-4;
    double force_regularization = 1.0e-6;
};

struct WbcObjective
{
    WbcHessian hessian = WbcHessian::Zero();
    WbcGradient gradient = WbcGradient::Zero();
};

struct WbcInequalitySystem
{
    static constexpr double kInfinity = 1.0e10;

    WbcInequalityMatrix matrix = WbcInequalityMatrix::Zero();

    WbcInequalityVector lower = WbcInequalityVector::Constant(-kInfinity);

    WbcInequalityVector upper = WbcInequalityVector::Constant(kInfinity);
};

// WBC只接收数值量，不直接依赖FSM或PinGo2Model，便于独立测试。
struct WbcInput
{
    WbcInput();

    MatX mass_matrix = MatX::Zero(WbcDimensions::kGeneralizedDof, WbcDimensions::kGeneralizedDof);
    Vec18 nonlinear_effects = Vec18::Zero(); // h(q,dq) = C(q,dq)dq + g(q)

    std::array<WbcFootJacobian, WbcDimensions::kNumLegs> foot_jacobians_world;
    std::array<Vec3, WbcDimensions::kNumLegs> foot_jdot_dq_world;

    VecInt4 contact = VecInt4::Ones();

    // 0表示尚未承载/已经卸载，1表示完整支撑；用于平滑缩放MPC力参考。
    Vec4 contact_activation = Vec4::Ones();

    // 地面对机器人的世界系接触力，与CentroidalMPC中的力定义一致。
    Vec34 mpc_contact_forces_world = Vec34::Zero();

    // 前6维必须遵循Pinocchio自由基座切空间的线速度/角速度顺序与坐标约定。
    Vec6 desired_base_acceleration_tangent = Vec6::Zero();
    Vec12 desired_joint_acceleration = Vec12::Zero();
    Vec34 desired_swing_foot_acceleration_world = Vec34::Zero();

    double friction_coefficient = 0.4;
    // 每腿独立上下限允许触地增力和离地卸力，而不改变QP稀疏结构。
    Vec4 min_normal_force = Vec4::Zero();
    Vec4 max_normal_force = Vec4::Constant(180.0);

    Vec12 torque_lower_bound = Vec12::Constant(-1.0e10);
    Vec12 torque_upper_bound = Vec12::Constant(1.0e10);
};

struct WbcOutput
{
    bool success = false;
    Vec18 generalized_acceleration = Vec18::Zero();
    Vec34 contact_forces_world = Vec34::Zero();
    Vec12 joint_torques = Vec12::Zero();

    double dynamics_residual_norm = std::numeric_limits<double>::infinity();
    double equality_residual_norm = std::numeric_limits<double>::infinity();
    double max_inequality_violation = std::numeric_limits<double>::infinity();

    std::string status;
};

class WbcController
{
  public:
    WbcController();

    bool validateInput(const WbcInput &input, std::string *error = nullptr) const;

    WbcEqualitySystem buildEqualitySystem(const WbcInput &input) const;

    WbcObjective buildObjective(const WbcInput &input, const WbcWeights &weights) const;

    WbcInequalitySystem buildInequalitySystem(const WbcInput &input) const;

    WbcOutput solve(const WbcInput &input, const WbcWeights &weights);

    std::size_t solverInitializationCount() const noexcept
    {
        return solver_initialization_count_;
    }

    Vec12 recoverJointTorques(const WbcInput &input, const Vec18 &generalized_acceleration,
                              const Vec34 &contact_forces_world) const;

    // 返回 M*ddq + h - S^T*tau - J_c^T*f。
    Vec18 computeDynamicsResidual(const WbcInput &input, const WbcOutput &output) const;

  private:
    using SolverConstraintMatrix =
        Eigen::Matrix<double, WbcDimensions::kSolverRows, WbcDimensions::kDecisionDof>;

    void initializeSparsePatterns();
    void updateSparseValues(const WbcObjective &objective,
                            const SolverConstraintMatrix &constraint_matrix);

    OsqpEigen::Solver solver_;
    bool solver_initialized_ = false;
    std::size_t solver_initialization_count_ = 0;
    bool has_previous_contact_ = false;
    VecInt4 previous_contact_ = VecInt4::Zero();

    Eigen::SparseMatrix<double> sparse_hessian_;
    Eigen::SparseMatrix<double> sparse_constraint_matrix_;

    VecX solver_gradient_ = VecX::Zero(WbcDimensions::kDecisionDof);
    VecX solver_lower_bound_ = VecX::Zero(WbcDimensions::kSolverRows);
    VecX solver_upper_bound_ = VecX::Zero(WbcDimensions::kSolverRows);
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER__CONTROL__WBC_CONTROLLER_H
