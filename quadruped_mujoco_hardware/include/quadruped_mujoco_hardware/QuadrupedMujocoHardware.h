#ifndef QUADRUPED_MUJOCO_HARDWARE__QUADRUPED_MUJOCO_HARDWARE_H
#define QUADRUPED_MUJOCO_HARDWARE__QUADRUPED_MUJOCO_HARDWARE_H

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <mujoco/mujoco.h>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace quadruped_mujoco_hardware
{

class QuadrupedMujocoHardware final : public hardware_interface::SystemInterface
{
  public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    ~QuadrupedMujocoHardware() override;

  private:
    void updateFootForces();
    void startViewer();
    void stopViewer();
    void viewerLoop();

    mjModel *model_{nullptr};
    mjData *data_{nullptr};
    std::mutex mujoco_mutex_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> imu_state_;
    std::vector<double> foot_force_state_;
    std::vector<double> odometer_state_;

    std::vector<int> joint_qpos_addr_;
    std::vector<int> joint_qvel_addr_;
    std::vector<int> actuator_id_;

    int imu_quat_sensor_id_{-1};
    int imu_gyro_sensor_id_{-1};
    int imu_acc_sensor_id_{-1};
    std::array<int, 4> foot_geom_ids_{{-1, -1, -1, -1}};

    double sim_timestep_{0.001};
    bool viewer_enabled_{false};
    double viewer_refresh_hz_{60.0};
    std::atomic_bool viewer_running_{false};
    std::thread viewer_thread_;
};

} // namespace quadruped_mujoco_hardware

#endif
