#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <stdexcept>

namespace quadruped_controller
{

QuadrupedRobot::QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                               const std::vector<std::string> &feet_names, const std::string &base_name,
                               const std::vector<std::string> &joint_names)
    : ctrl_interfaces_(ctrl_interfaces)
{
    go2_model_ = std::make_unique<go2_robot_data::PinGo2Model>(robot_description, feet_names, base_name);

    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        joint_state_index_by_name_[joint_names[i]] = i;
    }

    if (feet_names.size() != 4)
    {
        throw std::runtime_error("feet_names must contain exactly 4 frame names in FL, FR, RL, RR order");
    }

    const std::array<std::array<std::string, 3>, 4> leg_joint_names = {{
        {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"},
        {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"},
        {"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"},
        {"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"},
    }};

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int j = 0; j < 3; ++j)
        {
            const auto it = joint_state_index_by_name_.find(leg_joint_names[leg][j]);
            if (it == joint_state_index_by_name_.end())
            {
                throw std::runtime_error("joint_names is missing joint: " + leg_joint_names[leg][j]);
            }
            joint_state_indices_[leg][j] = it->second;
        }
    }

    mass_ = go2_model_->mass();

    // 5. 初始化
    current_joint_pos_.resize(4, Eigen::VectorXd::Zero(3));
    current_joint_vel_.resize(4, Eigen::VectorXd::Zero(3));
}

/// 从硬件接口更新当前关节位置/速度
void QuadrupedRobot::update()
{
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int j = 0; j < 3; ++j)
        {
            const std::size_t idx = joint_state_indices_[leg][j];
            current_joint_pos_[leg][j] = ctrl_interfaces_.joint_position_state_interface_[idx].get().get_value();
            current_joint_vel_[leg][j] = ctrl_interfaces_.joint_velocity_state_interface_[idx].get().get_value();
        }
    }

    go2_model_->updateModel(current_joint_pos_, current_joint_vel_);
}

void QuadrupedRobot::updatePinModelWithBase(const Vec3 &base_pos_world, const RotMat &base_rot_body_to_world,
                                            const Vec3 &base_linear_vel_world, const Vec3 &base_angular_vel_body)
{
    go2_model_->updateModelWithBase(base_pos_world, base_rot_body_to_world, base_linear_vel_world,
                                    base_angular_vel_body, current_joint_pos_, current_joint_vel_);
}

go2_robot_data::PinGo2Model &QuadrupedRobot::pinModel()
{
    return *go2_model_;
}

const go2_robot_data::PinGo2Model &QuadrupedRobot::pinModel() const
{
    return *go2_model_;
}

// ---- 足端位姿 ----
std::vector<SE3> QuadrupedRobot::getFeet2BPositions() const
{
    std::vector<SE3> result;
    for (int i = 0; i < 4; ++i)
    {
        result.push_back(getFeet2BPositions(i));
    }
    return result;
}

SE3 QuadrupedRobot::getFeet2BPositions(int index) const
{
    return go2_model_->footPoseBody(index);
}

// ---- 足端速度 ----
std::vector<Vec3> QuadrupedRobot::getFeet2BVelocities() const
{
    std::vector<Vec3> result;
    for (int i = 0; i < 4; ++i)
    {
        result.push_back(getFeet2BVelocities(i));
    }
    return result;
}

Vec3 QuadrupedRobot::getFeet2BVelocities(int index) const
{
    return go2_model_->footVelocityBody(index);
}

// ---- 雅可比 ----
Eigen::MatrixXd QuadrupedRobot::getJacobian(int index) const
{
    return go2_model_->footJacobianBody(index);
}

// ---- 逆运动学：足端位姿 → 关节角 ----
std::vector<Eigen::VectorXd> QuadrupedRobot::getQ(const std::vector<SE3> &foot_poses) const
{
    std::vector<Eigen::VectorXd> result;
    result.reserve(4);

    for (int i = 0; i < 4; ++i)
    {
        result.push_back(go2_model_->solveLegIKBody(i, foot_poses[i].translation(), current_joint_pos_[i]));
    }

    return result;
}

Vec12 QuadrupedRobot::getQ(const Vec34 &foot_positions) const
{
    Vec12 q = Vec12::Zero();

    for (int i = 0; i < 4; ++i)
    {
        q.segment(i * 3, 3) = go2_model_->solveLegIKBody(i, foot_positions.col(i), current_joint_pos_[i]);
    }

    return q;
}

Vec12 QuadrupedRobot::getQd(const std::vector<SE3> &foot_poses, const Vec34 &foot_vels)
{
    (void)foot_poses;

    Vec12 qd;
    // v = J * q̇  =>  q̇ = J^{-1} * v
    for (int i = 0; i < 4; ++i)
    {
        const Eigen::Matrix3d leg_jacobian = go2_model_->footJacobianBody(i);

        qd.segment(i * 3, 3) = leg_jacobian.colPivHouseholderQr().solve(foot_vels.col(i));
    }
    return qd;
}

// ---- 力矩：足端力 → 关节力矩 ----
Eigen::VectorXd QuadrupedRobot::getTorque(const Vec3 &force, int index) const
{
    const Eigen::Matrix3d leg_jacobian = go2_model_->footJacobianBody(index);
    Eigen::VectorXd leg_tau = leg_jacobian.transpose() * force;
    return leg_tau;
}

} // namespace quadruped_controller
