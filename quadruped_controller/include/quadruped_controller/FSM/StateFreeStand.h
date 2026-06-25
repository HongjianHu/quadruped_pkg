#ifndef STATEFREESTAND_H
#define STATEFREESTAND_H

#include "quadruped_controller/FSM/FSMState.h"

namespace quadruped_controller {

class StateFreeStand : public FSMState {
public:
  explicit StateFreeStand(CtrlInterfaces &ctrl_interfaces);

  void enter() override;
  void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  void exit() override;
  FSMStateName checkChange() override;
};

} // namespace quadruped_controller

#endif // STATEFREESTAND_H
