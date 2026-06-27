#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "quadruped_controller/common/mathTools.h"
#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/control/CtrlComponent.h"
#include "quadruped_controller/control/LowPassFilter.h"
#include "quadruped_controller/gait/WaveGenerator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

namespace quadruped_controller
{

struct CtrlInterfaces;
class WaveGenerator;
class QuadrupedRobot;
struct CtrlComponent;

class KalmanFilterEstimate
{
  public:
    KalmanFilterEstimate(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component);

    ~KalmanFilterEstimate() = default;
    /// 估计的身体位置（世界系）
    Vec3 getPosition()
    {
        return x_hat_.segment(0, 3);
    }

    /// 估计的身体速度（世界系）
    Vec3 getVelocity()
    {
        return x_hat_.segment(3, 3);
    }

    /// 估计的足端位置（世界系）
    Vec3 getFootPos(int index)
    {
        return getPosition() + rotation_ * foot_poses_[index].translation();
    }

    /// 四条足端位置（世界系）
    Vec34 getFeetPos()
    {
        Vec34 feet_pos;
        for (int i = 0; i < 4; ++i)
            feet_pos.col(i) = getFootPos(i);
        return feet_pos;
    }

    /// 四条足端速度（世界系）
    Vec34 getFeetVel()
    {
        const std::vector<Vec3> feet_vel = robot_model_->getFeet2BVelocities();
        Vec34 result;
        for (int i = 0; i < 4; ++i)
            result.col(i) = feet_vel[i] + getVelocity();
        return result;
    }

    /// 足端位置（身体系）
    Vec34 getFeetPos2Body()
    {
        Vec34 foot_pos;
        const Vec3 body_pos = getPosition();
        for (int i = 0; i < 4; i++)
            foot_pos.col(i) = getFootPos(i) - body_pos;
        return foot_pos;
    }

    RotMat getRotation()
    {
        return rotation_;
    }

    Vec3 getGyro()
    {
        return gyro_;
    }

    Vec3 getGyroGlobal() const
    {
        return rotation_ * gyro_;
    }

    double getYaw() const;

    double getDYaw() const
    {
        return getGyroGlobal()(2);
    }

    void update();

  private:
    CtrlInterfaces &ctrl_interfaces_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

    // 18 维状态：位置(3) + 速度(3) + 足端位置(3×4)
    Eigen::Matrix<double, 18, 1> x_hat_;
    Eigen::Matrix<double, 3, 1> u_;      // 输入
    Eigen::Matrix<double, 28, 1> y_;     // 测量值
    Eigen::Matrix<double, 28, 1> y_hat_; // 测量预测

    // 状态空间矩阵
    Eigen::Matrix<double, 18, 18> A; // 状态转移
    Eigen::Matrix<double, 18, 3> B;  // 输入矩阵
    Eigen::Matrix<double, 28, 18> C; // 输出矩阵

    // 协方差矩阵
    Eigen::Matrix<double, 18, 18> P;       // 估计协方差
    Eigen::Matrix<double, 18, 18> Ppriori; // 先验协方差
    Eigen::Matrix<double, 18, 18> Q;       // 过程噪声
    Eigen::Matrix<double, 28, 28> R;       // 测量噪声
    Eigen::Matrix<double, 18, 18> QInit_;  // Q 初始值
    Eigen::Matrix<double, 28, 28> RInit_;  // R 初始值
    Eigen::Matrix<double, 18, 1> Qdig;     // 可调过程噪声对角线
    Eigen::Matrix<double, 3, 3> Cu;        // 输入协方差

    // 测量中间量
    Eigen::Matrix<double, 12, 1> feet_pos_body_;
    Eigen::Matrix<double, 12, 1> feet_vel_body_;
    Eigen::Matrix<double, 4, 1> feet_h_;

    // EKF 中间变量
    Eigen::Matrix<double, 28, 28> S;
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> Slu;
    Eigen::Matrix<double, 28, 1> Sy;
    Eigen::Matrix<double, 28, 18> Sc;
    Eigen::Matrix<double, 28, 28> SR;
    Eigen::Matrix<double, 28, 18> STC;
    Eigen::Matrix<double, 18, 18> IKC;

    Vec3 g_;
    double dt_;

    RotMat rotation_;
    Vec3 acceleration_;
    Vec3 gyro_;

    std::vector<SE3> foot_poses_;
    std::vector<Vec3> foot_vels_;
    std::vector<std::shared_ptr<LowPassFilter>> low_pass_filters_;

    double large_variance_;
};

} // namespace quadruped_controller

#endif // ESTIMATOR_H