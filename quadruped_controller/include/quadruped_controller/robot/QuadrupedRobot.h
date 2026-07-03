#ifndef QUADRUPEDROBOT_H
#define QUADRUPEDROBOT_H

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "quadruped_controller/common/CtrlInterfaces.h"
#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/robot/go2_robot_data/PinGo2Model.h"

namespace quadruped_controller
{

class QuadrupedRobot
{
  public:
    QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                   const std::vector<std::string> &feet_names, const std::string &base_name,
                   const std::vector<std::string> &joint_names);

    /// 从硬件接口更新当前关节位置/速度
    void update();
    void updatePinModelWithBase(const Vec3 &base_pos_world, const RotMat &base_rot_body_to_world,
                                const Vec3 &base_linear_vel_world, const Vec3 &base_angular_vel_body);

    go2_robot_data::PinGo2Model &pinModel();
    const go2_robot_data::PinGo2Model &pinModel() const;

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

  private:
    CtrlInterfaces &ctrl_interfaces_;

    std::unique_ptr<go2_robot_data::PinGo2Model> go2_model_;
    std::unordered_map<std::string, std::size_t> joint_state_index_by_name_;
    std::array<std::array<std::size_t, 3>, 4> joint_state_indices_{};
};

} // namespace quadruped_controller

#endif // QUADRUPEDROBOT_H
