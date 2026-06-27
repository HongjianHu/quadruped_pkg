#include "quadruped_controller/QuadrupedController.h"

#include "quadruped_controller/gait/WaveGenerator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

namespace quadruped_controller
{

controller_interface::CallbackReturn QuadrupedController::on_init()
{
    try
    {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
        base_name_ = auto_declare<std::string>("base_name", base_name_);
        imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", imu_interface_types_);
        command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
        feet_names_ = auto_declare<std::vector<std::string>>("feet_names", feet_names_);

        down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
        stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
        stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
        stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);

        ctrl_interfaces_.frequency_ = auto_declare<int>("update_rate", ctrl_interfaces_.frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

        ctrl_component_.estimator_ = std::make_shared<KalmanFilterEstimate>(ctrl_interfaces_, ctrl_component_);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    control_input_subscription_ = get_node()->create_subscription<quadruped_controller_msgs::msg::Inputs>(
        "/control_input", 10, [this](const quadruped_controller_msgs::msg::Inputs::SharedPtr msg) {
            ctrl_interfaces_.control_inputs_.command = msg->command;
            ctrl_interfaces_.control_inputs_.lx = msg->lx;
            ctrl_interfaces_.control_inputs_.ly = msg->ly;
            ctrl_interfaces_.control_inputs_.rx = msg->rx;
            ctrl_interfaces_.control_inputs_.ry = msg->ry;
        });
    robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            ctrl_component_.robot_model_ =
                std::make_shared<QuadrupedRobot>(ctrl_interfaces_, msg->data, feet_names_, base_name_);
            ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
        });
    ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(0.45, 0.5, Vec4(0, 0.5, 0.5, 0));
    return CallbackReturn::SUCCESS;
}

using config_type = controller_interface::interface_configuration_type;

controller_interface::InterfaceConfiguration QuadrupedController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : command_interface_types_)
        {
            if (!command_prefix_.empty())
            {
                conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
            }
            else
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }
    }

    return conf;
}

controller_interface::InterfaceConfiguration QuadrupedController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : state_interface_types_)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

    for (const auto &interface_type : imu_interface_types_)
    {
        conf.names.push_back(imu_name_ + "/" + interface_type);
    }

    return conf;
}

controller_interface::CallbackReturn QuadrupedController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    ctrl_interfaces_.clear();

    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : command_interface_types_)
        {
            for (auto &interface : command_interfaces_)
            {
                if (interface.get_prefix_name() == joint_name && interface.get_interface_name() == interface_type)
                {
                    auto it = command_interface_map_.find(interface_type);
                    if (it != command_interface_map_.end())
                    {
                        it->second->push_back(interface);
                    }
                    break;
                }
            }
        }
    }

    for (const auto &joint_name : joint_names_)
    {
        for (const auto &interface_type : state_interface_types_)
        {
            for (auto &interface : state_interfaces_)
            {
                if (interface.get_prefix_name() == joint_name && interface.get_interface_name() == interface_type)
                {
                    auto it = state_interface_map_.find(interface_type);
                    if (it != state_interface_map_.end())
                    {
                        it->second->push_back(interface);
                    }
                    break;
                }
            }
        }
    }

    for (const auto &interface_type : imu_interface_types_)
    {
        for (auto &interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_ && interface.get_interface_name() == interface_type)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
                break;
            }
        }
    }

    state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
    state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
    state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
    state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
    state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_);

    current_state_ = state_list_.passive;
    current_state_->enter();
    next_state_ = current_state_;
    next_state_name_ = current_state_->state_name;
    mode_ = FSMMode::NORMAL;

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadrupedController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    if (ctrl_component_.robot_model_ == nullptr)
    {
        return controller_interface::return_type::OK;
    }
    ctrl_component_.robot_model_->update();
    ctrl_component_.wave_generator_->update();
    ctrl_component_.estimator_->update();

    if (mode_ == FSMMode::NORMAL)
    {
        current_state_->run(time, period);
        next_state_name_ = current_state_->checkChange();

        if (next_state_name_ != FSMStateName::INVALID && next_state_name_ != current_state_->state_name)
        {
            next_state_ = getNextState(next_state_name_);
            if (next_state_ != nullptr)
            {
                mode_ = FSMMode::CHANGE;
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
    }
    else if (mode_ == FSMMode::CHANGE)
    {
        current_state_->exit();
        current_state_ = next_state_;
        current_state_->enter();
        mode_ = FSMMode::NORMAL;
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn QuadrupedController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    release_interfaces();
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadrupedController::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

std::shared_ptr<FSMState> QuadrupedController::getNextState(FSMStateName stateName) const
{
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return state_list_.invalid;
    case FSMStateName::PASSIVE:
        return state_list_.passive;
    case FSMStateName::FIXEDDOWN:
        return state_list_.fixedDown;
    case FSMStateName::FIXEDSTAND:
        return state_list_.fixedStand;
    case FSMStateName::FREESTAND:
        return state_list_.freeStand;
    case FSMStateName::TROTTING:
        return state_list_.trotting;
    default:
        return state_list_.invalid;
    }
}
} // namespace quadruped_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_controller::QuadrupedController, controller_interface::ControllerInterface);