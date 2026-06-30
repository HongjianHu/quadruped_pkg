#include "quadruped_controller/robot/go2_robot_data/PinGo2Model.h"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>

#include <stdexcept>

namespace quadruped_controller
{
namespace go2_robot_data
{
PinGo2Model::PinGo2Model(const std::string &robot_description, const std::vector<std::string> &feet_names,
                         const std::string &base_name)
{
    pinocchio::urdf::buildModelFromXML(robot_description, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
    ik_data_ = pinocchio::Data(model_);

    base_frame_id_ = findFrame(base_name);

    foot_frame_ids_[0] = findFrame(findFootName(feet_names, "FL"));
    foot_frame_ids_[1] = findFrame(findFootName(feet_names, "FR"));
    foot_frame_ids_[2] = findFrame(findFootName(feet_names, "RL"));
    foot_frame_ids_[3] = findFrame(findFootName(feet_names, "RR"));

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
            const std::string &joint_name = leg_joint_names[leg][j];
            if (!model_.existJointName(joint_name))
            {
                throw std::runtime_error("PinGo2Model missing joint: " + joint_name);
            }

            const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
            leg_q_indices_[leg][j] = model_.idx_qs[joint_id];
            leg_v_indices_[leg][j] = model_.idx_vs[joint_id];
        }
    }

    q_ = pinocchio::neutral(model_);
    dq_ = Eigen::VectorXd::Zero(model_.nv);

    for (const auto &inertia : model_.inertias)
    {
        mass_ += inertia.mass();
    }

    updateModel(std::vector<Eigen::VectorXd>(4, Eigen::VectorXd::Zero(3)),
                std::vector<Eigen::VectorXd>(4, Eigen::VectorXd::Zero(3)));
}

double PinGo2Model::mass() const
{
    return mass_;
}

void PinGo2Model::updateModel(const std::vector<Eigen::VectorXd> &joint_pos,
                              const std::vector<Eigen::VectorXd> &joint_vel)
{
    if (joint_pos.size() != 4 || joint_vel.size() != 4)
    {
        throw std::runtime_error("PinGo2Model expects 4 legs");
    }

    q_ = pinocchio::neutral(model_);
    dq_.setZero();

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int j = 0; j < 3; ++j)
        {
            q_[leg_q_indices_[leg][j]] = joint_pos[leg][j];
            dq_[leg_v_indices_[leg][j]] = joint_vel[leg][j];
        }
    }

    pinocchio::forwardKinematics(model_, data_, q_, dq_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
}

SE3 PinGo2Model::footPoseBody(int index) const
{
    if (index < 0 || index >= 4)
    {
        throw std::runtime_error("PinGo2Model foot index out of range");
    }

    const SE3 &oMb = data_.oMf[base_frame_id_];
    const SE3 &oMf = data_.oMf[foot_frame_ids_[index]];
    return oMb.actInv(oMf);
}

Eigen::Matrix3d PinGo2Model::footJacobianBody(int index)
{
    if (index < 0 || index >= 4)
    {
        throw std::runtime_error("PinGo2Model foot index out of range");
    }

    Eigen::MatrixXd J_world = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data_, foot_frame_ids_[index], pinocchio::LOCAL_WORLD_ALIGNED, J_world);

    const Eigen::Matrix3d R_wb = data_.oMf[base_frame_id_].rotation();
    const Eigen::MatrixXd J_pos_body_full = R_wb.transpose() * J_world.topRows(3);

    Eigen::Matrix3d J_leg = Eigen::Matrix3d::Zero();
    for (int j = 0; j < 3; ++j)
    {
        J_leg.col(j) = J_pos_body_full.col(leg_v_indices_[index][j]);
    }

    return J_leg;
}

Vec3 PinGo2Model::footVelocityBody(int index)
{
    if (index < 0 || index >= 4)
    {
        throw std::runtime_error("PinGo2Model foot index out of range");
    }

    Vec3 qd_leg;
    for (int j = 0; j < 3; ++j)
    {
        qd_leg[j] = dq_[leg_v_indices_[index][j]];
    }

    return footJacobianBody(index) * qd_leg;
}

Eigen::VectorXd PinGo2Model::solveLegIKBody(int index, const Vec3 &target_foot_pos_body,
                                            const Eigen::VectorXd &initial_joint_pos)
{
    if (index < 0 || index >= 4)
    {
        throw std::runtime_error("PinGo2Model foot index out of range");
    }
    if (initial_joint_pos.size() != 3)
    {
        throw std::runtime_error("PinGo2Model IK expects a 3D initial joint vector");
    }

    const int max_iter = 8;
    const double eps = 1e-5;
    const double lambda = 1e-4;
    const double max_step = 0.2;

    Eigen::VectorXd q_work = q_;
    for (int j = 0; j < 3; ++j)
    {
        q_work[leg_q_indices_[index][j]] = initial_joint_pos[j];
    }

    for (int iter = 0; iter < max_iter; ++iter)
    {
        pinocchio::forwardKinematics(model_, ik_data_, q_work);
        pinocchio::computeJointJacobians(model_, ik_data_, q_work);
        pinocchio::updateFramePlacements(model_, ik_data_);

        const SE3 foot_body = ik_data_.oMf[base_frame_id_].actInv(ik_data_.oMf[foot_frame_ids_[index]]);
        const Vec3 err = target_foot_pos_body - foot_body.translation();

        if (err.norm() < eps)
        {
            break;
        }

        Eigen::MatrixXd J_world = Eigen::MatrixXd::Zero(6, model_.nv);
        pinocchio::getFrameJacobian(model_, ik_data_, foot_frame_ids_[index], pinocchio::LOCAL_WORLD_ALIGNED, J_world);

        const Eigen::Matrix3d R_wb = ik_data_.oMf[base_frame_id_].rotation();
        const Eigen::MatrixXd J_pos_body_full = R_wb.transpose() * J_world.topRows(3);

        Eigen::Matrix3d J_leg = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 3; ++j)
        {
            J_leg.col(j) = J_pos_body_full.col(leg_v_indices_[index][j]);
        }

        Eigen::Matrix3d JJT = J_leg * J_leg.transpose();
        JJT.diagonal().array() += lambda;

        Vec3 dq_leg = J_leg.transpose() * JJT.ldlt().solve(err);
        if (dq_leg.norm() > max_step)
        {
            dq_leg *= max_step / dq_leg.norm();
        }

        for (int j = 0; j < 3; ++j)
        {
            q_work[leg_q_indices_[index][j]] += dq_leg[j];
        }
    }

    Eigen::VectorXd leg_q = Eigen::VectorXd::Zero(3);
    for (int j = 0; j < 3; ++j)
    {
        leg_q[j] = q_work[leg_q_indices_[index][j]];
    }

    return leg_q;
}

// tool functions
std::string PinGo2Model::findFootName(const std::vector<std::string> &feet_names, const std::string &prefix)
{
    for (const auto &name : feet_names)
    {
        if (name.rfind(prefix + "_", 0) == 0)
        {
            return name;
        }
    }
    return prefix + "_foot";
}

pinocchio::FrameIndex PinGo2Model::findFrame(const std::string &frame_name) const
{
    if (model_.existFrame(frame_name))
    {
        return model_.getFrameId(frame_name);
    }
    if (model_.existFrame(frame_name + "_joint"))
    {
        return model_.getFrameId(frame_name + "_joint");
    }
    if (model_.existFrame(frame_name + "_fixed"))
    {
        return model_.getFrameId(frame_name + "_fixed");
    }
    throw std::runtime_error("PinGo2Model missing frame: " + frame_name);
}

} // namespace go2_robot_data
} // namespace quadruped_controller
