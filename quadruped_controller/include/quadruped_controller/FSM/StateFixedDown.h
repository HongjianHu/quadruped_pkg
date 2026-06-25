#ifndef STATEFIXEDDOWN_H
#define STATEFIXEDDOWN_H

#include "quadruped_controller/FSM/FSMState.h"
#include <vector>

namespace quadruped_controller {

class StateFixedDown : public FSMState {
public:
  explicit StateFixedDown(CtrlInterfaces &ctrl_interfaces,
                          const std::vector<double> &down_pos, double kp,
                          double kd);

  void enter() override;
  void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  void exit() override;
  FSMStateName checkChange() override;

private:
  std::vector<double> target_pos_;
  double kp_, kd_;
  int loop_count_;
  const int settle_time_ = 500; // 等待 500 个周期再允许切换
};

} // namespace quadruped_controller

#endif // STATEFIXEDDOWN_H
