#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <quadruped_controller_msgs/msg/inputs.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "quadruped_controller/FSM/FSMState.h"
#include "quadruped_controller/FSM/StateFixedDown.h"
#include "quadruped_controller/FSM/StateFixedStand.h"
#include "quadruped_controller/FSM/StateFreeStand.h"
#include "quadruped_controller/FSM/StateMPCTrotting.h"
#include "quadruped_controller/FSM/StateMPCWBCTrotting.h"
#include "quadruped_controller/FSM/StatePassive.h"
#include "quadruped_controller/common/CtrlInterfaces.h"
#include "quadruped_controller/control/CtrlComponent.h"
namespace quadruped_controller
{

// ---- FSM 状态列表 ----
struct FSMStateList
{
    std::shared_ptr<FSMState> invalid;
    std::shared_ptr<StatePassive> passive;
    std::shared_ptr<StateFixedDown> fixedDown;
    std::shared_ptr<StateFixedStand> fixedStand;
    std::shared_ptr<StateFreeStand> freeStand;
    std::shared_ptr<StateMPCTrotting> mpcTrotting;
    std::shared_ptr<StateMPCWBCTrotting> mpcWbcTrotting;
};

// ---- 控制器主类 ----
class QuadrupedController final : public controller_interface::ControllerInterface
{
  public:
    QuadrupedController() = default;

    // 1. controller_manager加载controller插件
    // 2. 调用 on_init(), controller进入unconfigured 状态
    controller_interface::CallbackReturn on_init() override;

    // 3. controller 进入 unconfigured 状态
    // 4. spawner 或 controller_manager configure controller

    // 5. 调用 on_configure() 这里通常 get_parameter()
    // 把yaml里的joints/command_interfaces/state_interfaces读入成员变量
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    // 6. controller_manager调用
    // 这一步会根据你刚刚读到的成员变量生成：FR_hip_joint/effort、FR_hip_joint/position
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // 7. controller_manager根据这些名字向ResourceManager申请接口
    // 如果hardware暴露了这些接口，就loan给controller
    // 8. 调用 on_activate()
    // 这里通常把command_interfaces_/state_interfaces_分配到ctrl_interfaces_结构里
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    // 9. controller active后周期调用update()
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    CtrlComponent ctrl_component_;
    CtrlInterfaces ctrl_interfaces_;

  protected:
    std::vector<std::string> joint_names_;             // robot_control.yaml
    std::vector<std::string> command_interface_types_; // robot_control.yaml
    std::vector<std::string> state_interface_types_;   // robot_control.yaml

    std::string imu_name_;                         // robot_control.yaml
    std::string base_name_;                        // robot_control.yaml
    std::string command_prefix_;                   // robot_control.yaml
    std::vector<std::string> imu_interface_types_; // robot_control.yaml
    std::string foot_force_name_ = "foot_force";
    std::vector<std::string> foot_force_interface_types_ = {"FL", "FR", "RL", "RR"};
    std::string odometer_name_ = "odometer";
    std::vector<std::string> odometer_interface_types_ = {"position.x", "position.y", "position.z",
                                                          "velocity.x", "velocity.y", "velocity.z"};
    std::vector<std::string> feet_names_; // robot_control.yaml

    // FL FR RL RR
    std::vector<double> stand_pos_ = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    std::vector<double> down_pos_ = {0.0, 1.3, -2.4, 0.0, 1.3, -2.4, 0.0, 1.3, -2.4, 0.0, 1.3, -2.4};

    double stand_kp_ = 80.0;
    double stand_kd_ = 3.5;

    rclcpp::Subscription<quadruped_controller_msgs::msg::Inputs>::SharedPtr
        control_input_subscription_; // 订阅用户动作命令的节点
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        robot_description_subscription_; // 根据传入的yaml文件将命令和状态接口读入读入成员变量
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr estimator_debug_publisher_;

    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
        command_interface_map_ = {{"effort", &ctrl_interfaces_.joint_torque_command_interface_},
                                  {"position", &ctrl_interfaces_.joint_position_command_interface_},
                                  {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
                                  {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
                                  {"kd", &ctrl_interfaces_.joint_kd_command_interface_}};

    FSMMode mode_ = FSMMode::NORMAL;
    std::string state_name_;
    FSMStateName next_state_name_ = FSMStateName::INVALID;
    FSMStateList state_list_;
    std::shared_ptr<FSMState> current_state_;
    std::shared_ptr<FSMState> next_state_;

    std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
    double update_frequency_;

    std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;
    void publishEstimatorDebug(const rclcpp::Time &time);

    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        state_interface_map_ = {{"position", &ctrl_interfaces_.joint_position_state_interface_},
                                {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
                                {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}};
};
} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER_H
