#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <pinocchio/multibody/data.hpp>

#include <stdexcept>

namespace quadruped_controller
{

QuadrupedRobot::QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                               const std::vector<std::string> &feet_names, const std::string &base_name)
    : ctrl_interfaces_(ctrl_interfaces)
{
    (void)base_name;

    pinocchio::urdf::buildModelFromXML(robot_description, model_);
    data_fr_ = pinocchio::Data(model_);
    data_fl_ = pinocchio::Data(model_);
    data_rr_ = pinocchio::Data(model_);
    data_rl_ = pinocchio::Data(model_);

    const std::array<std::array<std::string, 3>, 4> leg_joint_names = {{
        {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"},
        {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"},
        {"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"},
        {"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"},
    }};

    leg_q_indices_.resize(4);
    leg_v_indices_.resize(4);

    // 腿顺序：FR=0, FL=1, RR=2, RL=3
    // 这里必须按关节名查询 Pinocchio 的 q/v 索引，不能假设它等于 controller/YAML 的顺序。
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int j = 0; j < 3; ++j)
        {
            const std::string &joint_name = leg_joint_names[leg][j];
            if (!model_.existJointName(joint_name))
            {
                throw std::runtime_error("Pinocchio model is missing joint: " + joint_name);
            }

            const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
            leg_q_indices_[leg][j] = model_.idx_qs[joint_id];
            leg_v_indices_[leg][j] = model_.idx_vs[joint_id];
        }
    }

    // 3. 创建四条腿(腿的顺序不应该是觉得顺序，后面可以优化一下)
    for (int i = 0; i < 4; ++i)
    {
        std::vector<pinocchio::JointIndex> joint_ids;
        for (const auto &jn : leg_joint_names[i])
        {
            if (model_.existJointName(jn))
                joint_ids.push_back(model_.getJointId(jn));
        }

        pinocchio::FrameIndex foot_fid = 0;
        if (!feet_names.empty() && model_.existFrame(feet_names[i]))
            foot_fid = model_.getFrameId(feet_names[i]);
        else
            foot_fid = model_.getFrameId(feet_names[i] + "_foot");

        pinocchio::Data *leg_data = (i == 0 ? &data_fr_ : i == 1 ? &data_fl_ : i == 2 ? &data_rr_ : &data_rl_);
        robot_legs_.push_back(std::make_shared<RobotLeg>(model_, *leg_data, joint_ids, foot_fid));
    }

    for (auto &inertia : model_.inertias)
        mass_ += inertia.mass();

    // 5. 初始化
    current_joint_pos_.resize(4, Eigen::VectorXd::Zero(3));
    current_joint_vel_.resize(4, Eigen::VectorXd::Zero(3));
    q_full_ = Eigen::VectorXd::Zero(model_.nq);
}

/// 从硬件接口更新当前关节位置/速度
void QuadrupedRobot::update()
{
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int j = 0; j < 3; ++j)
        {
            const int idx = 3 * leg + j;
            current_joint_pos_[leg][j] = ctrl_interfaces_.joint_position_state_interface_[idx].get().get_value();
            current_joint_vel_[leg][j] = ctrl_interfaces_.joint_velocity_state_interface_[idx].get().get_value();
        }
    }

    q_full_.setZero();
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int j = 0; j < 3; ++j)
        {
            q_full_[leg_q_indices_[leg][j]] = current_joint_pos_[leg][j];
        }
    }
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
    return robot_legs_[index]->calcPEe2B(q_full_);
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
    Eigen::MatrixXd J = robot_legs_[index]->calcJaco(q_full_);
    // v_foot = J * q̇_full
    Eigen::VectorXd qd_full = Eigen::VectorXd::Zero(model_.nv);
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int j = 0; j < 3; ++j)
        {
            qd_full[leg_v_indices_[leg][j]] = current_joint_vel_[leg][j];
        }
    }
    return J * qd_full;
}

// ---- 雅可比 ----
Eigen::MatrixXd QuadrupedRobot::getJacobian(int index) const
{
    return robot_legs_[index]->calcJaco(q_full_);
}

// ---- 逆运动学：足端位姿 → 关节角 ----
std::vector<Eigen::VectorXd> QuadrupedRobot::getQ(const std::vector<SE3> &foot_poses) const
{
    std::vector<Eigen::VectorXd> result;
    for (int i = 0; i < 4; ++i)
    {
        const Eigen::VectorXd q_solution = robot_legs_[i]->calcQ(foot_poses[i], q_full_);
        Eigen::VectorXd leg_q = Eigen::VectorXd::Zero(3);
        for (int j = 0; j < 3; ++j)
        {
            leg_q[j] = q_solution[leg_q_indices_[i][j]];
        }
        result.push_back(leg_q);
    }
    return result;
}
Vec12 QuadrupedRobot::getQ(const Vec34 &foot_positions) const
{
    Vec12 q;
    for (int i = 0; i < 4; ++i)
    {
        SE3 target = SE3::Identity();
        target.translation() = foot_positions.col(i);
        Eigen::VectorXd qi = robot_legs_[i]->calcQ(target, q_full_);
        // 只取本腿的 3 个关节
        for (int j = 0; j < 3; ++j)
        {
            q[i * 3 + j] = qi[leg_q_indices_[i][j]];
        }
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
        Eigen::MatrixXd J = robot_legs_[i]->calcJaco(q_full_); // 3 * model_nv
        Eigen::MatrixXd leg_jacobian(3, 3);
        for (int j = 0; j < 3; ++j)
        {
            leg_jacobian.col(j) = J.col(leg_v_indices_[i][j]);
        }

        qd.segment(i * 3, 3) = leg_jacobian.colPivHouseholderQr().solve(foot_vels.col(i));
    }
    return qd;
}

// ---- 力矩：足端力 → 关节力矩 ----
Eigen::VectorXd QuadrupedRobot::getTorque(const Vec3 &force, int index) const
{
    return robot_legs_[index]->calcTorque(q_full_, force);
}

} // namespace quadruped_controller
