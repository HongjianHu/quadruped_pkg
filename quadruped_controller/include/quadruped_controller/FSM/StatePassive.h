#ifndef STATEPASSIVE_H
#define STATEPASSIVE_H

#include "quadruped_controller/FSM/FSMState.h"

namespace quadruped_controller
{

class StatePassive : public FSMState
{
  public:
    explicit StatePassive(CtrlInterfaces &ctrl_interfaces);

    void enter() override;
    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    void exit() override;
};

} // namespace quadruped_controller

#endif // STATEPASSIVE_H
