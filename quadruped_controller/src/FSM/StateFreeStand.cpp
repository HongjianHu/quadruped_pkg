#include "quadruped_controller/FSM/StateFreeStand.h"

#include "quadruped_controller/common/mathTools.h"
#include "quadruped_controller/control/CtrlComponent.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <cmath>
namespace quadruped_controller
{

StateFreeStand::StateFreeStand(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::FREESTAND, "FREESTAND", ctrl_interfaces), robot_model_(ctrl_component.robot_model_),
      roll_max_(20.0 * M_PI / 180.0), roll_min_(-roll_max_), pitch_max_(15.0 * M_PI / 180.0), pitch_min_(-pitch_max_),
      yaw_max_(20.0 * M_PI / 180.0), yaw_min_(-yaw_max_), height_max_(0.1), height_min_(-height_max_),
      fr_init_pos_(SE3::Identity())
{
}

void StateFreeStand::enter()
{
    if (!robot_model_)
    {
        return;
    }

    init_joint_torque_.assign(4, Eigen::VectorXd::Zero(3));

    for (std::size_t i = 0; i < 12; ++i)
    {
        const int leg = static_cast<int>(i / 3);
        const int joint = static_cast<int>(i % 3);
        if (i < ctrl_interfaces_.joint_effort_state_interface_.size())
        {
            init_joint_torque_[leg][joint] = ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value();
        }

        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(init_joint_torque_[leg][joint]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(100.0);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(5.0);

        if (i < ctrl_interfaces_.joint_position_state_interface_.size())
        {
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
                ctrl_interfaces_.joint_position_state_interface_[i].get().get_value());
        }
    }

    init_joint_pos_ = robot_model_->current_joint_pos_;
    init_foot_pos_ = robot_model_->getFeet2BPositions();

    fr_init_pos_ = init_foot_pos_[0];
    for (auto &foot_pos : init_foot_pos_)
    {
        foot_pos.translation() -= fr_init_pos_.translation();
        foot_pos.rotation().setIdentity();
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateFreeStand::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!robot_model_)
    {
        return;
    }

    calc_body_target(invNormalize(ctrl_interfaces_.control_inputs_.lx, roll_min_, roll_max_),
                     invNormalize(ctrl_interfaces_.control_inputs_.ly, pitch_min_, pitch_max_),
                     invNormalize(ctrl_interfaces_.control_inputs_.rx, yaw_min_, yaw_max_),
                     invNormalize(ctrl_interfaces_.control_inputs_.ry, height_min_, height_max_));
}

void StateFreeStand::exit()
{
}

FSMStateName StateFreeStand::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDSTAND;
    case 3:
        return FSMStateName::TROTTING;
    default:
        return FSMStateName::FREESTAND;
    }
}

void StateFreeStand::calc_body_target(float roll, float pitch, float yaw, float height)
{
    SE3 fr_to_body = SE3::Identity();
    fr_to_body.translation() = -fr_init_pos_.translation();
    fr_to_body.translation().z() += height;

    Eigen::AngleAxisd roll_rot(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_rot(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rot(-yaw, Eigen::Vector3d::UnitZ());

    fr_to_body.rotation() = (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();

    const SE3 body_to_fr = fr_to_body.inverse();

    std::vector<SE3> goal_foot_poses(4, SE3::Identity());

    for (int i = 0; i < 4; ++i)
    {
        goal_foot_poses[i] = body_to_fr * init_foot_pos_[i]; //目标坐标系下，foot相对于body的位姿
    }

    target_joint_pos_ = robot_model_->getQ(goal_foot_poses);

    for (int leg = 0; leg < 4; ++leg)
    {
        const Eigen::VectorXd &leg_q = target_joint_pos_[leg];

        for (int j = 0; j < 3; ++j)
        {
            const int idx = leg * 3 + j;
            ctrl_interfaces_.joint_position_command_interface_[idx].get().set_value(leg_q[j]);
            if (leg < static_cast<int>(init_joint_torque_.size()))
            {
                ctrl_interfaces_.joint_torque_command_interface_[idx].get().set_value(init_joint_torque_[leg][j]);
            }
        }
    }
}
} // namespace quadruped_controller
