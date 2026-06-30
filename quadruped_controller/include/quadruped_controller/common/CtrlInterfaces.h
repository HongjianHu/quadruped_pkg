#ifndef CTRL_INTERFACES_H
#define CTRL_INTERFACES_H

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <quadruped_controller_msgs/msg/inputs.hpp>
#include <vector>

namespace quadruped_controller
{

struct CtrlInterfaces
{
    // ---- 指令接口（你写给硬件）----
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_kd_command_interface_;

    // ---- 状态接口（硬件回报给你）----
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

    // ---- IMU ----
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> imu_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> foot_force_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> odometer_state_interface_;

    // ---- 控制输入（键盘发来的信号）----
    quadruped_controller_msgs::msg::Inputs control_inputs_;
    int frequency_{200};

    void clear()
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kp_command_interface_.clear();
        joint_kd_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
        foot_force_state_interface_.clear();
        odometer_state_interface_.clear();
    }
};

} // namespace quadruped_controller

#endif // CTRL_INTERFACES_H
