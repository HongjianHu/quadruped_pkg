#ifndef STATETROTTING_H
#define STATETROTTING_H

#include "quadruped_controller/FSM/FSMState.h"
#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/gait/GaitGenerator.h"

#include <vector>

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
    static constexpr double kJointKd = 3.0;
    static constexpr double kGaitHeight = 0.06;
    static constexpr double kEntryStanceDuration = 0.25;
    static constexpr double kEntryRampDuration = 0.45;
    static constexpr double kSupportBlendDuration = 0.25;
    static constexpr double kHipTorqueLimit = 23.0;
    static constexpr double kThighTorqueLimit = 23.0;
    static constexpr double kCalfTorqueLimit = 35.0;
    static constexpr double kBaseKpZ = 28.0;
    static constexpr double kBaseKdZ = 7.0;
    static constexpr double kBaseKpXY = 3.0;
    static constexpr double kBaseKdXY = 2.0;
    static constexpr double kBaseKpRP = 45.0;
    static constexpr double kBaseKdRP = 8.0;
    static constexpr double kBaseKpYaw = 12.0;
    static constexpr double kBaseKdYaw = 3.0;

    static double smoothStep(double x);
    static double smoothStepDerivative(double x);
    static double clampJointTorque(double torque, int joint);
    static double clampValue(double value, double limit);

    CtrlComponent &ctrl_component_;
    GaitGenerator gait_generator_;
    std::vector<double> stand_feedforward_torque_;
    Vec34 entry_feet_body_;
    Vec12 entry_joint_pos_;
    Vec3 desired_base_pos_;
    double desired_yaw_{0.0};
    double elapsed_time_{0.0};
    double next_balance_debug_time_{0.0};
    bool gait_started_{false};
};

} // namespace quadruped_controller

#endif // STATETROTTING_H
