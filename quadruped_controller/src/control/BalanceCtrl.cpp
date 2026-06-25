#include "quadruped_controller/control/BalanceCtrl.h"

#include <qpOASES.hpp>

#include "quadruped_controller/common/mathTools.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

namespace quadruped_controller {

BalanceCtrl::BalanceCtrl(const std::shared_ptr<QuadrupedRobot> &robot) {
  mass_ = robot->mass_;

  alpha_ = 0.001;
  beta_ = 0.1;
  g_ << 0, 0, -9.81;
  friction_ratio_ = 0.4;

  // 线性化摩擦锥：fx, fy 受 μ·fz 约束，fz ≥ 0
  // 5 行：fx≤μfz, -fx≤μfz, fy≤μfz, -fy≤μfz, fz≥0
  friction_mat_ << 1, 0, friction_ratio_, -1, 0, friction_ratio_, 0, 1,
      friction_ratio_, 0, -1, friction_ratio_, 0, 0, 1;

  // 质心偏移（默认在身体系原点）
  pcb_ = Vec3(0.0, 0.0, 0.0);

  // 身体惯性（硬编码，后续可从 URDF 读取）
  Ib_ = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();

  // ---- QP 权重 ----
  Vec6 s;
  s << 20, 20, 50, 450, 450, 450;
  S_ = s.asDiagonal();

  Vec12 w;
  w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
  W_ = w.asDiagonal();

  Vec12 u;
  u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
  U_ = u.asDiagonal();

  F_prev_.setZero();
}

Vec34 BalanceCtrl::calF(const Vec3 &ddPcd, const Vec3 &dWbd,
                        const RotMat &rot_matrix, const Vec34 &feet_pos_2_body,
                        const VecInt4 &contact) {
  calMatrixA(feet_pos_2_body, rot_matrix);
  calVectorBd(ddPcd, dWbd, rot_matrix);
  calConstraints(contact);

  // 二次型：G = AᵀSA + αW + βU
  G_ = A_.transpose() * S_ * A_ + alpha_ * W_ + beta_ * U_;

  // 线性项（行向量）：g0ᵀ = -bdᵀ·S·A - β·F_prevᵀ·U
  g0T_ = -bd_.transpose() * S_ * A_ - beta_ * F_prev_.transpose() * U_;

  solveQP();

  F_prev_ = F_;
  return vec12ToVec34(F_);
}

void BalanceCtrl::calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM) {
  for (int i = 0; i < 4; ++i) {
    // 力部分：每个足端力直接累加到合力
    A_.block(0, 3 * i, 3, 3) = Mat3::Identity();

    // 力矩部分：r × F，力臂 = 足端位置 - 质心（世界系）
    Vec3 lever_arm = Vec3(feet_pos_2_body.col(i)) - rotM * pcb_;
    A_.block(3, 3 * i, 3, 3) = skew(lever_arm);
  }
}

void BalanceCtrl::calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd,
                              const RotMat &rotM) {
  // 期望合力 = m·(a_des - g)
  bd_.head(3) = mass_ * (ddPcd - g_);

  // 期望合力矩 = R·I_b·Rᵀ·α_des（世界系）
  bd_.tail(3) = rotM * Ib_ * rotM.transpose() * dWbd;
}

void BalanceCtrl::calConstraints(const VecInt4 &contact) {
  int contactLegNum = 0;
  for (int i = 0; i < 4; ++i)
    if (contact[i] == 1)
      contactLegNum++;

  CI_.resize(5 * contactLegNum, 12);
  ci0_.resize(5 * contactLegNum);
  CE_.resize(3 * (4 - contactLegNum), 12);
  ce0_.resize(3 * (4 - contactLegNum));

  CI_.setZero();
  ci0_.setZero();
  CE_.setZero();
  ce0_.setZero();

  int ceID = 0;
  int ciID = 0;
  for (int i = 0; i < 4; ++i) {
    if (contact[i] == 1) {
      // 触地腿：摩擦锥不等式约束
      // CIᵀ·F + ci0 ≥ 0  →  每腿 5 个不等式
      CI_.block(5 * ciID, 3 * i, 5, 3) = friction_mat_;
      ++ciID;
    } else {
      // 离地腿：力为零的等式约束
      // CEᵀ·F + ce0 = 0  →  F_i = 0
      CE_.block(3 * ceID, 3 * i, 3, 3) = Mat3::Identity();
      ++ceID;
    }
  }
}

void BalanceCtrl::solveQP() {
  const int nV = 12;
  const int nEq = static_cast<int>(ce0_.size());
  const int nIneq = static_cast<int>(ci0_.size());
  const int nC = nEq + nIneq;

  std::vector<qpOASES::real_t> H(nV * nV);
  for (int i = 0; i < nV; ++i)
    for (int j = 0; j < nV; ++j)
      H[i * nV + j] = G_(i, j);

  std::vector<qpOASES::real_t> g(nV);
  for (int i = 0; i < nV; ++i)
    g[i] = g0T_(i);

  std::vector<qpOASES::real_t> A(nC * nV);
  for (int i = 0; i < nEq; ++i)
    for (int j = 0; j < nV; ++j)
      A[i * nV + j] = CE_(i, j);
  for (int i = 0; i < nIneq; ++i)
    for (int j = 0; j < nV; ++j)
      A[(nEq + i) * nV + j] = CI_(i, j);

  std::vector<qpOASES::real_t> lb(nV), ub(nV);
  for (int i = 0; i < nV; ++i) {
    lb[i] = -qpOASES::INFTY;
    ub[i] = qpOASES::INFTY;
  }

  std::vector<qpOASES::real_t> lbA(nC), ubA(nC);

  for (int i = 0; i < nEq; ++i)
    lbA[i] = ubA[i] = -ce0_(i);

  for (int i = 0; i < nIneq; ++i) {
    lbA[nEq + i] = -ci0_(i);
    ubA[nEq + i] = qpOASES::INFTY;
  }

  // ---- 求解 ----
  qpOASES::QProblem qp(nV, nC);
  qpOASES::Options options;
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);

  int nWSR = 100;
  qpOASES::returnValue status =
      qp.init(H.data(), g.data(), A.data(), lb.data(), ub.data(), lbA.data(),
              ubA.data(), nWSR);

  if (status != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "[BalanceCtrl] qpOASES failed: " << status << std::endl;
    return;
  }

  std::vector<qpOASES::real_t> x(nV);
  qp.getPrimalSolution(x.data());
  for (int i = 0; i < nV; ++i)
    F_(i) = x[i];
}

} // namespace quadruped_controller