#include "quadruped_controller/QuadrupedController.h"

#include "quadruped_controller/common/mathTools.h"
#include "quadruped_controller/gait/WaveGenerator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"

#include <limits>

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
        foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
        foot_force_interface_types_ =
            auto_declare<std::vector<std::string>>("foot_force_interfaces", foot_force_interface_types_);
        odometer_name_ = auto_declare<std::string>("odometer_name", odometer_name_);
        odometer_interface_types_ =
            auto_declare<std::vector<std::string>>("odometer_interfaces", odometer_interface_types_);
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
                std::make_shared<QuadrupedRobot>(ctrl_interfaces_, msg->data, feet_names_, base_name_, joint_names_);
        });
    estimator_debug_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/estimator_debug", 10);
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

    for (const auto &interface_type : foot_force_interface_types_)
    {
        conf.names.push_back(foot_force_name_ + "/" + interface_type);
    }

    for (const auto &interface_type : odometer_interface_types_)
    {
        conf.names.push_back(odometer_name_ + "/" + interface_type);
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

    for (const auto &interface_type : foot_force_interface_types_)
    {
        for (auto &interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == foot_force_name_ && interface.get_interface_name() == interface_type)
            {
                ctrl_interfaces_.foot_force_state_interface_.emplace_back(interface);
                break;
            }
        }
    }

    for (const auto &interface_type : odometer_interface_types_)
    {
        for (auto &interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == odometer_name_ && interface.get_interface_name() == interface_type)
            {
                ctrl_interfaces_.odometer_state_interface_.emplace_back(interface);
                break;
            }
        }
    }

    state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
    state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
    state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
    state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_);
    state_list_.mpcTrotting = std::make_shared<StateMPCTrotting>(ctrl_interfaces_, ctrl_component_);

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
    publishEstimatorDebug(time);

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

void QuadrupedController::publishEstimatorDebug(const rclcpp::Time &time)
{
    if (estimator_debug_publisher_ == nullptr || ctrl_component_.estimator_ == nullptr ||
        ctrl_component_.wave_generator_ == nullptr)
    {
        return;
    }

    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    const Vec3 est_pos = ctrl_component_.estimator_->getPosition();
    const Vec3 est_vel = ctrl_component_.estimator_->getVelocity();
    const Vec3 rpy = rotMatToRPY(ctrl_component_.estimator_->getRotation());
    const Vec3 acc_body = ctrl_component_.estimator_->getAccelerationBody();
    const Vec3 u_world = ctrl_component_.estimator_->getAccelerationWorldInput();
    const Vec34 feet_pos_world = ctrl_component_.estimator_->getFeetPos();
    const Vec34 feet_contact_base_to_world = ctrl_component_.estimator_->getFeetContactPosBaseToWorld();
    const VecInt4 force_contact = ctrl_component_.estimator_->getForceContact();
    const VecInt4 slip_detected = ctrl_component_.estimator_->getSlipDetected();
    const VecInt4 estimator_contact = ctrl_component_.estimator_->getEstimatorContact();

    Vec3 odom_pos = Vec3::Constant(kNaN);
    Vec3 odom_vel = Vec3::Constant(kNaN);
    if (ctrl_interfaces_.odometer_state_interface_.size() >= 6)
    {
        odom_pos << ctrl_interfaces_.odometer_state_interface_[0].get().get_value(),
            ctrl_interfaces_.odometer_state_interface_[1].get().get_value(),
            ctrl_interfaces_.odometer_state_interface_[2].get().get_value();
        odom_vel << ctrl_interfaces_.odometer_state_interface_[3].get().get_value(),
            ctrl_interfaces_.odometer_state_interface_[4].get().get_value(),
            ctrl_interfaces_.odometer_state_interface_[5].get().get_value();
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "t,est_pos3,est_vel3,rpy3,acc_body3,u_world3,odom_pos3,odom_vel3,feet_pos_world12,foot_force4,"
        "force_contact4,slip_detected4,est_contact4,wave_contact4,phase4,pos_err3,vel_err3,"
        "odom_contact_pos_world12,odom_contact_clearance4";
    msg.layout.dim[0].size = 80;
    msg.layout.dim[0].stride = 80;
    msg.data.reserve(80);

    msg.data.push_back(time.seconds());

    for (int i = 0; i < 3; ++i)
        msg.data.push_back(est_pos[i]);
    for (int i = 0; i < 3; ++i)
        msg.data.push_back(est_vel[i]);
    for (int i = 0; i < 3; ++i)
        msg.data.push_back(rpy[i]);
    for (int i = 0; i < 3; ++i)
        msg.data.push_back(acc_body[i]);
    for (int i = 0; i < 3; ++i)
        msg.data.push_back(u_world[i]);
    for (int i = 0; i < 3; ++i)
        msg.data.push_back(odom_pos[i]);
    for (int i = 0; i < 3; ++i)
        msg.data.push_back(odom_vel[i]);

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            msg.data.push_back(feet_pos_world(axis, leg));
        }
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        if (leg < static_cast<int>(ctrl_interfaces_.foot_force_state_interface_.size()))
        {
            msg.data.push_back(ctrl_interfaces_.foot_force_state_interface_[leg].get().get_value());
        }
        else
        {
            msg.data.push_back(kNaN);
        }
    }

    for (int leg = 0; leg < 4; ++leg)
        msg.data.push_back(force_contact[leg]);
    for (int leg = 0; leg < 4; ++leg)
        msg.data.push_back(slip_detected[leg]);
    for (int leg = 0; leg < 4; ++leg)
        msg.data.push_back(estimator_contact[leg]);
    for (int leg = 0; leg < 4; ++leg)
        msg.data.push_back(ctrl_component_.wave_generator_->contact_[leg]);
    for (int leg = 0; leg < 4; ++leg)
        msg.data.push_back(ctrl_component_.wave_generator_->phase_[leg]);

    for (int i = 0; i < 3; ++i)
        msg.data.push_back(est_pos[i] - odom_pos[i]);
    for (int i = 0; i < 3; ++i)
        msg.data.push_back(est_vel[i] - odom_vel[i]);

    for (int leg = 0; leg < 4; ++leg)
    {
        const Vec3 contact_pos_world = odom_pos + feet_contact_base_to_world.col(leg);
        for (int axis = 0; axis < 3; ++axis)
        {
            msg.data.push_back(contact_pos_world[axis]);
        }
    }
    for (int leg = 0; leg < 4; ++leg)
    {
        const Vec3 contact_pos_world = odom_pos + feet_contact_base_to_world.col(leg);
        msg.data.push_back(contact_pos_world.z());
    }

    estimator_debug_publisher_->publish(msg);
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
    case FSMStateName::MPC_TROTTING:
        return state_list_.mpcTrotting;
    default:
        return state_list_.invalid;
    }
}
} // namespace quadruped_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_controller::QuadrupedController, controller_interface::ControllerInterface);
