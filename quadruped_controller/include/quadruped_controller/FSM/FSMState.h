#ifndef FSMSTATE_H
#define FSMSTATE_H

#include "quadruped_controller/common/CtrlInterfaces.h"
#include "quadruped_controller/common/enumClass.h"
#include <rclcpp/time.hpp>
#include <string>
#include <utility>

namespace quadruped_controller {

class FSMState {
public:
  virtual ~FSMState() = default;
  FSMState(const FSMStateName &state_name, std::string state_name_string,
           CtrlInterfaces &ctrl_interfaces)
      : state_name(state_name), state_name_string(std::move(state_name_string)),
        ctrl_interfaces_(ctrl_interfaces) {}

  virtual void enter() = 0;
  virtual void run(const rclcpp::Time &time,
                   const rclcpp::Duration &period) = 0;
  virtual void exit() = 0;

  virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

  FSMStateName state_name;
  std::string state_name_string;

protected:
  CtrlInterfaces &ctrl_interfaces_;
};

} // namespace quadruped_controller

#endif // FSMSTATE_H