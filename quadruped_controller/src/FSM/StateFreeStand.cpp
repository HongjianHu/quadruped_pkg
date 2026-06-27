#include "quadruped_controller/FSM/StateFreeStand.h"

#include "quadruped_controller/control/CtrlComponent.h"

namespace quadruped_controller
{

StateFreeStand::StateFreeStand(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component)
    : FSMState(FSMStateName::FREESTAND, "FREESTAND", ctrl_interfaces), ctrl_component_(ctrl_component)
{
}

void StateFreeStand::enter()
{
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateFreeStand::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    (void)ctrl_component_;
}

void StateFreeStand::exit()
{
}

FSMStateName StateFreeStand::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDSTAND;
    case 3:
        return FSMStateName::TROTTING;
    default:
        return FSMStateName::FREESTAND;
    }
}

} // namespace quadruped_controller
