#ifndef STATEFREESTAND_H
#define STATEFREESTAND_H

#include "quadruped_controller/FSM/FSMState.h"
#include "quadruped_controller/common/mathTypes.h"

#include <memory>
#include <vector>
namespace quadruped_controller
{

class QuadrupedRobot;
struct CtrlComponent;

class StateFreeStand : public FSMState
{
  public:
    explicit StateFreeStand(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component);

    void enter() override;
    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    void exit() override;
    FSMStateName checkChange() override;

  private:
    std::shared_ptr<QuadrupedRobot> &robot_model_;

    void calc_body_target(float roll, float pitch, float yaw, float height);

    float roll_max_, roll_min_;
    float pitch_max_, pitch_min_;
    float yaw_max_, yaw_min_;
    float height_max_, height_min_;

    std::vector<Eigen::VectorXd> init_joint_pos_;
    std::vector<Eigen::VectorXd> init_joint_torque_;
    std::vector<Eigen::VectorXd> target_joint_pos_;

    SE3 fr_init_pos_;
    std::vector<SE3> init_foot_pos_;
};

} // namespace quadruped_controller

#endif // STATEFREESTAND_H
