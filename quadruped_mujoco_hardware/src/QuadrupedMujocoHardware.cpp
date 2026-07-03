#include "quadruped_mujoco_hardware/QuadrupedMujocoHardware.h"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <thread>

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

bool parseBoolParam(const std::string &value)
{
    return value == "true" || value == "True" || value == "TRUE" || value == "1";
}

struct ViewerMouseState
{
    const mjModel *model{nullptr};
    mjvCamera *camera{nullptr};
    mjvScene *scene{nullptr};
    double last_x{0.0};
    double last_y{0.0};
    bool has_last_position{false};
};

ViewerMouseState *getViewerMouseState(GLFWwindow *window)
{
    return static_cast<ViewerMouseState *>(glfwGetWindowUserPointer(window));
}

void viewerMouseButtonCallback(GLFWwindow *window, int /*button*/, int /*action*/, int /*mods*/)
{
    auto *state = getViewerMouseState(window);
    if (state == nullptr)
    {
        return;
    }

    glfwGetCursorPos(window, &state->last_x, &state->last_y);
    state->has_last_position = true;
}

void viewerCursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
    auto *state = getViewerMouseState(window);
    if (state == nullptr)
    {
        return;
    }

    if (!state->has_last_position)
    {
        state->last_x = xpos;
        state->last_y = ypos;
        state->has_last_position = true;
        return;
    }

    const double dx = xpos - state->last_x;
    const double dy = ypos - state->last_y;
    state->last_x = xpos;
    state->last_y = ypos;

    if (state->model == nullptr || state->camera == nullptr || state->scene == nullptr)
    {
        return;
    }

    const bool left_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool middle_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    const bool right_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (!left_button && !middle_button && !right_button)
    {
        return;
    }

    const bool shift_pressed =
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    int action = mjMOUSE_NONE;
    if (middle_button)
    {
        action = mjMOUSE_ZOOM;
    }
    else if (right_button)
    {
        action = shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (left_button)
    {
        action = shift_pressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0)
    {
        return;
    }

    mjv_moveCamera(state->model, action, dx / static_cast<double>(height), dy / static_cast<double>(height),
                   state->scene, state->camera);
}

void viewerScrollCallback(GLFWwindow *window, double /*xoffset*/, double yoffset)
{
    auto *state = getViewerMouseState(window);
    if (state == nullptr || state->model == nullptr || state->camera == nullptr || state->scene == nullptr)
    {
        return;
    }

    mjv_moveCamera(state->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, state->scene, state->camera);
}

} // namespace

QuadrupedMujocoHardware::~QuadrupedMujocoHardware()
{
    stopViewer();

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

    const auto viewer_it = info_.hardware_parameters.find("enable_viewer");
    if (viewer_it != info_.hardware_parameters.end())
    {
        viewer_enabled_ = parseBoolParam(viewer_it->second);
    }

    const auto viewer_rate_it = info_.hardware_parameters.find("viewer_refresh_hz");
    if (viewer_rate_it != info_.hardware_parameters.end())
    {
        viewer_refresh_hz_ = std::max(1.0, std::stod(viewer_rate_it->second));
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

    const std::array<std::string, 4> foot_geom_names = {"FL", "FR", "RL", "RR"};
    for (std::size_t i = 0; i < foot_geom_names.size(); ++i)
    {
        foot_geom_ids_[i] = mj_name2id(model_, mjOBJ_GEOM, foot_geom_names[i].c_str());
        if (foot_geom_ids_[i] < 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("quadruped_mujoco_hardware"),
                        "Foot geom '%s' not found; its foot_force interface will stay zero",
                        foot_geom_names[i].c_str());
        }
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
                "Loaded MuJoCo model '%s': nq=%ld nv=%ld nu=%ld joints=%zu timestep=%.6f viewer=%s", model_path.c_str(),
                static_cast<long>(model_->nq), static_cast<long>(model_->nv), static_cast<long>(model_->nu),
                joint_count, model_->opt.timestep, viewer_enabled_ ? "true" : "false");

    if (viewer_enabled_)
    {
        startViewer();
    }

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
    std::lock_guard<std::mutex> lock(mujoco_mutex_);

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

    updateFootForces();

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
    std::lock_guard<std::mutex> lock(mujoco_mutex_);

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

void QuadrupedMujocoHardware::updateFootForces()
{
    std::fill(foot_force_state_.begin(), foot_force_state_.end(), 0.0);

    if (model_ == nullptr || data_ == nullptr)
    {
        return;
    }

    const std::size_t foot_count = std::min<std::size_t>(foot_force_state_.size(), foot_geom_ids_.size());
    for (int contact_id = 0; contact_id < data_->ncon; ++contact_id)
    {
        const mjContact &contact = data_->contact[contact_id];
        mjtNum contact_force[6] = {0, 0, 0, 0, 0, 0};
        mj_contactForce(model_, data_, contact_id, contact_force);
        const double normal_force = std::max(0.0, static_cast<double>(contact_force[0]));

        for (std::size_t foot = 0; foot < foot_count; ++foot)
        {
            const int geom_id = foot_geom_ids_[foot];
            if (geom_id >= 0 && (contact.geom1 == geom_id || contact.geom2 == geom_id))
            {
                foot_force_state_[foot] += normal_force;
            }
        }
    }
}

void QuadrupedMujocoHardware::startViewer()
{
    if (viewer_running_.load())
    {
        return;
    }

    viewer_running_.store(true);
    viewer_thread_ = std::thread(&QuadrupedMujocoHardware::viewerLoop, this);
}

void QuadrupedMujocoHardware::stopViewer()
{
    viewer_running_.store(false);
    if (viewer_thread_.joinable())
    {
        viewer_thread_.join();
    }
}

void QuadrupedMujocoHardware::viewerLoop()
{
    const auto logger = rclcpp::get_logger("quadruped_mujoco_hardware");

    if (!glfwInit())
    {
        RCLCPP_WARN(logger, "Failed to initialize GLFW; embedded MuJoCo viewer is disabled");
        viewer_running_.store(false);
        return;
    }

    GLFWwindow *window = glfwCreateWindow(1280, 900, "quadruped embedded MuJoCo", nullptr, nullptr);
    if (window == nullptr)
    {
        RCLCPP_WARN(logger, "Failed to create GLFW window; embedded MuJoCo viewer is disabled");
        glfwTerminate();
        viewer_running_.store(false);
        return;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    camera.type = mjCAMERA_FREE;
    camera.azimuth = 90.0;
    camera.elevation = -20.0;
    camera.distance = 3.0;
    camera.lookat[0] = 0.0;
    camera.lookat[1] = 0.0;
    camera.lookat[2] = 0.25;

    ViewerMouseState mouse_state;
    mouse_state.model = model_;
    mouse_state.camera = &camera;
    mouse_state.scene = &scene;
    glfwSetWindowUserPointer(window, &mouse_state);
    glfwSetMouseButtonCallback(window, viewerMouseButtonCallback);
    glfwSetCursorPosCallback(window, viewerCursorPosCallback);
    glfwSetScrollCallback(window, viewerScrollCallback);

    {
        std::lock_guard<std::mutex> lock(mujoco_mutex_);
        if (model_ != nullptr)
        {
            mjv_makeScene(model_, &scene, 2000);
            mjr_makeContext(model_, &context, mjFONTSCALE_150);
        }
    }

    const auto frame_period = std::chrono::duration<double>(1.0 / std::max(1.0, viewer_refresh_hz_));
    RCLCPP_INFO(logger,
                "Embedded MuJoCo viewer started. Controls: left-drag rotate, right-drag pan, shift+drag alternate "
                "plane, middle-drag/wheel zoom");

    while (viewer_running_.load() && !glfwWindowShouldClose(window))
    {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        const mjrRect viewport = {0, 0, width, height};

        {
            std::lock_guard<std::mutex> lock(mujoco_mutex_);
            if (model_ != nullptr && data_ != nullptr)
            {
                mjv_updateScene(model_, data_, &option, nullptr, &camera, mjCAT_ALL, &scene);
            }
        }

        mjr_render(viewport, &scene, &context);
        glfwSwapBuffers(window);
        glfwPollEvents();
        std::this_thread::sleep_for(frame_period);
    }

    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwSetWindowUserPointer(window, nullptr);
    glfwDestroyWindow(window);
    glfwTerminate();
    viewer_running_.store(false);
    RCLCPP_INFO(logger, "Embedded MuJoCo viewer stopped");
}

} // namespace quadruped_mujoco_hardware

PLUGINLIB_EXPORT_CLASS(quadruped_mujoco_hardware::QuadrupedMujocoHardware, hardware_interface::SystemInterface)
