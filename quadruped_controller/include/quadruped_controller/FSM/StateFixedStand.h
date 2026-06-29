#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include "quadruped_controller/FSM/FSMState.h"
#include <vector>

namespace quadruped_controller
{

class StateFixedStand : public FSMState
{
  public:
    explicit StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &stand_pos, double kp,
                             double kd);

    void enter() override;
    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    void exit() override;
    FSMStateName checkChange() override;

  private:
    std::vector<double> target_pos_;
    std::vector<double> init_pos_;
    double kp_, kd_;
    int loop_count_;
    double elapsed_time_{0.0};
    static constexpr double kStandRampDuration = 2.0;
};

} // namespace quadruped_controller

#endif // STATEFIXEDSTAND_H
