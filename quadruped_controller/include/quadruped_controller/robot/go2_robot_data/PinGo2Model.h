#ifndef PIN_GO2_MODEL_H
#define PIN_GO2_MODEL_H

#include <array>
#include <string>
#include <vector>

#include "quadruped_controller/common/mathTypes.h"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
namespace quadruped_controller
{
namespace go2_robot_data
{

class PinGo2Model
{
  public:
    PinGo2Model(const std::string &robot_description, const std::vector<std::string> &feet_names,
                const std::string &base_name);

    double mass() const;

    void updateModel(const std::vector<Eigen::VectorXd> &joint_pos, const std::vector<Eigen::VectorXd> &joint_vel);

    SE3 footPoseBody(int index) const;

    Vec3 footVelocityBody(int index);

    Eigen::Matrix3d footJacobianBody(int index);

    struct FootStateWorld
    {
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
    };

    struct DynamicsTerms
    {
        VecX g;
        MatX C;
        MatX M;
    };

    Vec3 footPositionWorld(int index) const;
    Vec3 footVelocityWorld(int index);
    FootStateWorld footStateWorld(int index);

    Eigen::Matrix3d footJacobianWorld(int index);
    Eigen::MatrixXd fullFootJacobianWorld(int index);

    Vec3 computeJdotDqWorld(int index);
    DynamicsTerms computeDynamicsTerms();

    const VecX &q() const;
    const VecX &dq() const;

    Vec3 legJointVector(const VecX &generalized_vector, int index) const;

    Eigen::VectorXd solveLegIKBody(int index, const Vec3 &target_foot_pos_body,
                                   const Eigen::VectorXd &initial_joint_pos);

  private:
    static std::string findFootName(const std::vector<std::string> &feet_names, const std::string &prefix);
    pinocchio::FrameIndex findFrame(const std::string &frame_name) const;

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::Data ik_data_;
    pinocchio::FrameIndex base_frame_id_{0};
    std::array<pinocchio::FrameIndex, 4> foot_frame_ids_{};
    std::array<std::array<int, 3>, 4> leg_q_indices_{};
    std::array<std::array<int, 3>, 4> leg_v_indices_{};
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    double mass_{0.0};
};

} // namespace go2_robot_data
} // namespace quadruped_controller

#endif // PIN_GO2_MODEL_H
