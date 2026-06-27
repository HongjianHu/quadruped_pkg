#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <memory>

#include "quadruped_controller/common/mathTypes.h"

namespace quadruped_controller
{

class QuadrupedRobot;

class BalanceCtrl
{
  public:
    explicit BalanceCtrl(const std::shared_ptr<QuadrupedRobot> &robot);

    ~BalanceCtrl() = default;

    /// 计算四足期望接触力
    /// @param ddPcd 期望身体线加速度（世界系）
    /// @param dWbd  期望身体角加速度（世界系）
    /// @param rot_matrix 当前身体旋转矩阵
    /// @param feet_pos_2_body 足端在身体系下的位置（世界系坐标）
    /// @param contact 接触状态 [FR, FL, RR, RL]，1=触地
    /// @return 3×4 足端力矩阵
    Vec34 calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix, const Vec34 &feet_pos_2_body,
               const VecInt4 &contact);

  private:
    void calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM);
    void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM);
    void calConstraints(const VecInt4 &contact);
    void solveQP();

    // 正则化权重矩阵
    Mat12 G_, W_, U_;
    Mat6 S_;

    // 动力学参数
    Mat3 Ib_;  // 身体惯性矩阵（身体系）
    Vec6 bd_;  // 期望合力/合力矩 (6维)
    Vec3 g_;   // 重力
    Vec3 pcb_; // 质心偏移（身体系）

    // QP 变量
    Vec12 F_;      // 当前解
    Vec12 F_prev_; // 上一周期解（平滑）
    Vec12 g0T_;    // QP 成本函数线性项

    double mass_;
    double alpha_, beta_;
    double friction_ratio_;

    // 约束
    Eigen::Matrix<double, 6, 12> A_;           // 力→合力/矩映射
    Eigen::Matrix<double, 5, 3> friction_mat_; // 摩擦锥线性化 (5×3)

    // QP 约束矩阵 (动态大小，取决于触地腿数量)
    Eigen::MatrixXd CE_, CI_;
    Eigen::VectorXd ce0_, ci0_;
};

} // namespace quadruped_controller

#endif // BALANCECTRL_H
