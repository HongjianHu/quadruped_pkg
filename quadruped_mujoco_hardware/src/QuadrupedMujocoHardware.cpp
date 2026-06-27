#include "quadruped_mujoco_hardware/QuadrupedMujocoHardware.h"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace quadruped_mujoco_hardware
{

namespace
{

int findActuatorForJoint(const mjModel *model, int joint_id)
{
    for (int i = 0; i < model->nu; ++i)
    {
        if (model->actuator_trntype[i] == mjTRN_JOINT && model->actuator_trnid[2 * i] == joint_id)
        {
            return i;
        }
    }
    return -1;
}

double clampActuatorCommand(const mjModel *model, int actuator_id, double command)
{
    if (model->actuator_ctrllimited[actuator_id] == 0)
    {
        return command;
    }

    const double min_cmd = model->actuator_ctrlrange[2 * actuator_id];
    const double max_cmd = model->actuator_ctrlrange[2 * actuator_id + 1];
    return std::clamp(command, min_cmd, max_cmd);
}

} // namespace

QuadrupedMujocoHardware::~QuadrupedMujocoHardware()
{
    if (data_ != nullptr)
    {
        mj_deleteData(data_);
        data_ = nullptr;
    }

    if (model_ != nullptr)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}

hardware_interface::CallbackReturn QuadrupedMujocoHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto model_path_it = info_.hardware_parameters.find("model_path");
    if (model_path_it == info_.hardware_parameters.end() || model_path_it->second.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("quadruped_mujoco_hardware"), "Missing hardware parameter: model_path");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string model_path = model_path_it->second;

    const auto timestep_it = info_.hardware_parameters.find("sim_timestep");
    if (timestep_it != info_.hardware_parameters.end())
    {
        sim_timestep_ = std::stod(timestep_it->second);
    }

    char error[1024] = "";
    model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (model_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("quadruped_mujoco_hardware"), "Failed to load MuJoCo model: %s, error: %s",
                     model_path.c_str(), error);
        return hardware_interface::CallbackReturn::ERROR;
    }

    model_->opt.timestep = sim_timestep_;

    data_ = mj_makeData(model_);
    if (data_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("quadruped_mujoco_hardware"), "Failed to allocate MuJoCo data");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto keyframe_it = info_.hardware_parameters.find("initial_keyframe");
    if (keyframe_it != info_.hardware_parameters.end() && !keyframe_it->second.empty())
    {
        const int key_id = mj_name2id(model_, mjOBJ_KEY, keyframe_it->second.c_str());
        if (key_id >= 0)
        {
            mj_resetDataKeyframe(model_, data_, key_id);
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("quadruped_mujoco_hardware"),
                        "Initial keyframe '%s' not found, using default reset", keyframe_it->second.c_str());
            mj_resetData(model_, data_);
        }
    }
    else
    {
        mj_resetData(model_, data_);
    }

    mj_forward(model_, data_);

    const std::size_t joint_count = info_.joints.size();

    joint_position_.assign(joint_count, 0.0);
    joint_velocity_.assign(joint_count, 0.0);
    joint_effort_.assign(joint_count, 0.0);

    joint_position_command_.assign(joint_count, 0.0);
    joint_velocity_command_.assign(joint_count, 0.0);
    joint_effort_command_.assign(joint_count, 0.0);
    joint_kp_command_.assign(joint_count, 0.0);
    joint_kd_command_.assign(joint_count, 0.0);

    joint_qpos_addr_.assign(joint_count, -1);
    joint_qvel_addr_.assign(joint_count, -1);
    actuator_id_.assign(joint_count, -1);

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        const std::string &joint_name = info_.joints[i].name;
        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_name.c_str());

        if (joint_id < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("quadruped_mujoco_hardware"),
                         "Joint '%s' exists in ros2_control but not in MuJoCo model", joint_name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        const int actuator_id = findActuatorForJoint(model_, joint_id);
        if (actuator_id < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("quadruped_mujoco_hardware"), "No MuJoCo actuator found for joint '%s'",
                         joint_name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        joint_qpos_addr_[i] = model_->jnt_qposadr[joint_id];
        joint_qvel_addr_[i] = model_->jnt_dofadr[joint_id];
        actuator_id_[i] = actuator_id;

        joint_position_[i] = data_->qpos[joint_qpos_addr_[i]];
        joint_velocity_[i] = data_->qvel[joint_qvel_addr_[i]];

        joint_position_command_[i] = joint_position_[i];
        joint_velocity_command_[i] = 0.0;
        joint_effort_command_[i] = 0.0;
        joint_kp_command_[i] = 0.0;
        joint_kd_command_[i] = 0.0;
    }

    imu_state_.assign(10, 0.0);
    imu_state_[0] = 1.0;

    imu_quat_sensor_id_ = mj_name2id(model_, mjOBJ_SENSOR, "imu_quat");
    imu_gyro_sensor_id_ = mj_name2id(model_, mjOBJ_SENSOR, "imu_gyro");
    imu_acc_sensor_id_ = mj_name2id(model_, mjOBJ_SENSOR, "imu_acc");

    if (imu_quat_sensor_id_ < 0 || imu_gyro_sensor_id_ < 0 || imu_acc_sensor_id_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("quadruped_mujoco_hardware"),
                     "MuJoCo model must provide imu_quat, imu_gyro and imu_acc sensors");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.sensors.size() > 1)
    {
        foot_force_state_.assign(info_.sensors[1].state_interfaces.size(), 0.0);
    }
    else
    {
        foot_force_state_.assign(4, 0.0);
    }

    if (info_.sensors.size() > 2)
    {
        odometer_state_.assign(info_.sensors[2].state_interfaces.size(), 0.0);
    }
    else
    {
        odometer_state_.assign(6, 0.0);
    }
    RCLCPP_INFO(rclcpp::get_logger("quadruped_mujoco_hardware"),
                "Loaded MuJoCo model '%s': nq=%ld nv=%ld nu=%ld joints=%zu timestep=%.6f", model_path.c_str(),
                static_cast<long>(model_->nq), static_cast<long>(model_->nv), static_cast<long>(model_->nu),
                joint_count, model_->opt.timestep);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QuadrupedMujocoHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        interfaces.emplace_back(info_.joints[i].name, "position", &joint_position_[i]);
        interfaces.emplace_back(info_.joints[i].name, "velocity", &joint_velocity_[i]);
        interfaces.emplace_back(info_.joints[i].name, "effort", &joint_effort_[i]);
    }

    if (!info_.sensors.empty())
    {
        for (std::size_t i = 0; i < info_.sensors[0].state_interfaces.size(); ++i)
        {
            interfaces.emplace_back(info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_state_[i]);
        }
    }

    if (info_.sensors.size() > 1)
    {
        for (std::size_t i = 0; i < info_.sensors[1].state_interfaces.size(); ++i)
        {
            interfaces.emplace_back(info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name,
                                    &foot_force_state_[i]);
        }
    }

    if (info_.sensors.size() > 2)
    {
        for (std::size_t i = 0; i < info_.sensors[2].state_interfaces.size(); ++i)
        {
            interfaces.emplace_back(info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name,
                                    &odometer_state_[i]);
        }
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface> QuadrupedMujocoHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        interfaces.emplace_back(info_.joints[i].name, "position", &joint_position_command_[i]);
        interfaces.emplace_back(info_.joints[i].name, "velocity", &joint_velocity_command_[i]);
        interfaces.emplace_back(info_.joints[i].name, "effort", &joint_effort_command_[i]);
        interfaces.emplace_back(info_.joints[i].name, "kp", &joint_kp_command_[i]);
        interfaces.emplace_back(info_.joints[i].name, "kd", &joint_kd_command_[i]);
    }

    return interfaces;
}

hardware_interface::return_type QuadrupedMujocoHardware::read(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/)
{
    if (model_ == nullptr || data_ == nullptr)
    {
        return hardware_interface::return_type::ERROR;
    }

    mj_forward(model_, data_);

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        joint_position_[i] = data_->qpos[joint_qpos_addr_[i]];
        joint_velocity_[i] = data_->qvel[joint_qvel_addr_[i]];
        joint_effort_[i] = data_->actuator_force[actuator_id_[i]];
    }

    const int quat_adr = model_->sensor_adr[imu_quat_sensor_id_];
    const int gyro_adr = model_->sensor_adr[imu_gyro_sensor_id_];
    const int acc_adr = model_->sensor_adr[imu_acc_sensor_id_];

    imu_state_[0] = data_->sensordata[quat_adr + 0];
    imu_state_[1] = data_->sensordata[quat_adr + 1];
    imu_state_[2] = data_->sensordata[quat_adr + 2];
    imu_state_[3] = data_->sensordata[quat_adr + 3];

    imu_state_[4] = data_->sensordata[gyro_adr + 0];
    imu_state_[5] = data_->sensordata[gyro_adr + 1];
    imu_state_[6] = data_->sensordata[gyro_adr + 2];

    imu_state_[7] = data_->sensordata[acc_adr + 0];
    imu_state_[8] = data_->sensordata[acc_adr + 1];
    imu_state_[9] = data_->sensordata[acc_adr + 2];

    for (double &force : foot_force_state_)
    {
        force = 0.0;
    }

    if (odometer_state_.size() >= 6)
    {
        odometer_state_[0] = data_->qpos[0];
        odometer_state_[1] = data_->qpos[1];
        odometer_state_[2] = data_->qpos[2];

        odometer_state_[3] = data_->qvel[0];
        odometer_state_[4] = data_->qvel[1];
        odometer_state_[5] = data_->qvel[2];
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuadrupedMujocoHardware::write(const rclcpp::Time & /*time*/,
                                                               const rclcpp::Duration &period)
{
    if (model_ == nullptr || data_ == nullptr)
    {
        return hardware_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
        const double q = data_->qpos[joint_qpos_addr_[i]];
        const double dq = data_->qvel[joint_qvel_addr_[i]];

        double torque = joint_effort_command_[i] + joint_kp_command_[i] * (joint_position_command_[i] - q) +
                        joint_kd_command_[i] * (joint_velocity_command_[i] - dq);

        if (!std::isfinite(torque))
        {
            torque = 0.0;
        }

        data_->ctrl[actuator_id_[i]] = clampActuatorCommand(model_, actuator_id_[i], torque);
    }

    int steps = 1;
    if (period.seconds() > 0.0 && model_->opt.timestep > 0.0)
    {
        steps = std::max(1, static_cast<int>(std::round(period.seconds() / model_->opt.timestep)));
    }

    for (int i = 0; i < steps; ++i)
    {
        mj_step(model_, data_);
    }

    return hardware_interface::return_type::OK;
}

} // namespace quadruped_mujoco_hardware

PLUGINLIB_EXPORT_CLASS(quadruped_mujoco_hardware::QuadrupedMujocoHardware, hardware_interface::SystemInterface)