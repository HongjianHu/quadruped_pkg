#ifndef STATETROTTING_H
#define STATETROTTING_H

#include "quadruped_controller/FSM/FSMState.h"

namespace quadruped_controller
{

struct CtrlComponent;

class StateTrotting : public FSMState
{
  public:
    explicit StateTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component);

    void enter() override;
    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    void exit() override;
    FSMStateName checkChange() override;

  private:
    CtrlComponent &ctrl_component_;
};

} // namespace quadruped_controller

#endif // STATETROTTING_H
