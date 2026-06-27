#include "quadruped_controller/robot/RobotLeg.h"

namespace quadruped_controller
{

RobotLeg::RobotLeg(const pinocchio::Model &model, pinocchio::Data &data,
                   const std::vector<pinocchio::JointIndex> &joint_ids, pinocchio::FrameIndex foot_frame_id)
    : model_(model), data_(data), joint_ids_(joint_ids), foot_frame_id_(foot_frame_id)
{
}

SE3 RobotLeg::calcPEe2B(const Eigen::VectorXd &q_full)
{
    pinocchio::forwardKinematics(model_, data_, q_full);
    pinocchio::updateFramePlacement(model_, data_, foot_frame_id_);
    return data_.oMf[foot_frame_id_];
}

Eigen::MatrixXd RobotLeg::calcJaco(const Eigen::VectorXd &q_full)
{
    pinocchio::computeJointJacobians(model_, data_, q_full);

    Eigen::MatrixXd J_6xN = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data_, foot_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_6xN);
    return J_6xN.topRows(3);
}

Eigen::VectorXd RobotLeg::calcQ(const SE3 &target_pose, const Eigen::VectorXd &q_full)
{
    const int max_iter = 200;
    const double eps = 1e-4;
    const double lambda = 0.5;

    Eigen::VectorXd q = q_full;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        SE3 current_pose = calcPEe2B(q);
        SE3 error = target_pose.actInv(current_pose);
        Eigen::Matrix<double, 6, 1> err_vec = pinocchio::log6(error).toVector();

        if (err_vec.norm() < eps)
            break;

        pinocchio::computeJointJacobians(model_, data_, q);

        Eigen::MatrixXd J_6xN = Eigen::MatrixXd::Zero(6, model_.nv);
        pinocchio::getFrameJacobian(model_, data_, foot_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_6xN);
        Eigen::MatrixXd JJT = J_6xN * J_6xN.transpose();
        JJT.diagonal() += Eigen::VectorXd::Constant(6, lambda);

        Eigen::VectorXd dq = J_6xN.transpose() * JJT.ldlt().solve(err_vec);

        q += dq;
    }

    return q;
}

Eigen::VectorXd RobotLeg::calcTorque(const Eigen::VectorXd &q_full, const Vec3 &foot_force)
{
    Eigen::MatrixXd J_3xN = calcJaco(q_full);
    return J_3xN.transpose() * foot_force;
}
} // namespace quadruped_controller
