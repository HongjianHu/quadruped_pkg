#ifndef STATETROTTING_H
#define STATETROTTING_H

#include "quadruped_controller/FSM/FSMState.h"
#include "quadruped_controller/common/mathTypes.h"

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
    static constexpr double kJointKp = 45.0;
    static constexpr double kJointKd = 1.5;
    static constexpr double kGaitHeight = 0.025;
    static constexpr double kSupportExtension = 0.05;
    static constexpr double kBodyXCompensation = -0.025;

    CtrlComponent &ctrl_component_;
    Vec34 init_feet_body_;
    bool initialized_{false};
};

} // namespace quadruped_controller

#endif // STATETROTTING_H
