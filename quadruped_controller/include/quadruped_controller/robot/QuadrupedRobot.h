#ifndef QUADRUPEDROBOT_H
#define QUADRUPEDROBOT_H

#include <memory>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>
#include <vector>

#include "RobotLeg.h"
#include "quadruped_controller/common/CtrlInterfaces.h"
#include "quadruped_controller/common/mathTypes.h"
namespace quadruped_controller {

class QuadrupedRobot {
public:
  QuadrupedRobot(CtrlInterfaces &ctrl_interfaces,
                 const std::string &robot_description,
                 const std::vector<std::string> &feet_names,
                 const std::string &base_name);

  /// 从硬件接口更新当前关节位置/速度
  void update();

  // ---- 足端位姿 ----
  std::vector<SE3> getFeet2BPositions() const;
  SE3 getFeet2BPositions(int index) const;

  // ---- 足端速度 ----
  std::vector<Vec3> getFeet2BVelocities() const;
  Vec3 getFeet2BVelocities(int index) const;

  // ---- 雅可比 ----
  Eigen::MatrixXd getJacobian(int index) const;

  // ---- 逆运动学：足端位姿 → 关节角 ----
  std::vector<Eigen::VectorXd> getQ(const std::vector<SE3> &foot_poses) const;
  Vec12 getQ(const Vec34 &foot_positions) const;
  Vec12 getQd(const std::vector<SE3> &foot_poses, const Vec34 &foot_vels);

  // ---- 力矩：足端力 → 关节力矩 ----
  Eigen::VectorXd getTorque(const Vec3 &force, int index) const;

  // ---- 机器人物性 ----
  double mass_ = 0.0;
  Vec34 feet_pos_normal_stand_;

  // ---- 当前关节状态 ----
  std::vector<Eigen::VectorXd> current_joint_pos_;
  std::vector<Eigen::VectorXd> current_joint_vel_;
  Eigen::VectorXd q_full_;

private:
  CtrlInterfaces &ctrl_interfaces_;
  std::vector<std::shared_ptr<RobotLeg>> robot_legs_;

  // Pinocchio 模型（只有一份，四条腿共享引用）
  pinocchio::Model model_;
  // 每条腿一个 data，避免并发覆盖
  pinocchio::Data data_fr_, data_fl_, data_rr_, data_rl_;
};

} // namespace quadruped_controller

#endif // QUADRUPEDROBOT_H