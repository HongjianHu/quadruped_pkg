#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include "quadruped_controller/common/mathTypes.h"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace quadruped_controller
{

class RobotLeg
{
  public:
    explicit RobotLeg(const pinocchio::Model &model, pinocchio::Data &data,
                      const std::vector<pinocchio::JointIndex> &joint_ids, pinocchio::FrameIndex foot_frame_id);

    SE3 calcPEe2B(const Eigen::VectorXd &q_full);

    Eigen::VectorXd calcQ(const SE3 &target_pose, const Eigen::VectorXd &q_full);

    Eigen::MatrixXd calcJaco(const Eigen::VectorXd &q_full);

    Eigen::VectorXd calcTorque(const Eigen::VectorXd &q_full, const Vec3 &foot_force);

  private:
    const pinocchio::Model &model_;
    pinocchio::Data &data_;
    std::vector<pinocchio::JointIndex> joint_ids_;
    pinocchio::FrameIndex foot_frame_id_;
};

} // namespace quadruped_controller

#endif // ROBOTLEG_H