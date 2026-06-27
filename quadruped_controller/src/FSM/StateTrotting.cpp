#include "quadruped_controller/FSM/StateTrotting.h"

#include "quadruped_controller/control/CtrlComponent.h"

namespace quadruped_controller
{

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::TROTTING, "TROTTING", ctrl_interfaces), ctrl_component_(ctrl_component)
{
}

void StateTrotting::enter()
{
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateTrotting::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    (void)ctrl_component_;
}

void StateTrotting::exit()
{
}

FSMStateName StateTrotting::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDSTAND;
    default:
        return FSMStateName::TROTTING;
    }
}

} // namespace quadruped_controller
