#include "quadruped_controller/FSM/StatePassive.h"

namespace quadruped_controller
{

StatePassive::StatePassive(CtrlInterfaces &ctrl_interfaces)
    : FSMState(FSMStateName::PASSIVE, "PASSIVE", ctrl_interfaces)
{
}

void StatePassive::enter()
{
    for (auto &cmd : ctrl_interfaces_.joint_torque_command_interface_)
        cmd.get().set_value(0.0);
}

void StatePassive::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (auto &cmd : ctrl_interfaces_.joint_torque_command_interface_)
        cmd.get().set_value(0.0);
}

void StatePassive::exit()
{
}

FSMStateName StatePassive::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 2:
        return FSMStateName::FIXEDSTAND;
    default:
        return FSMStateName::PASSIVE;
    }
}
} // namespace quadruped_controller
