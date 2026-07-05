#ifndef STATEMPCTROTTING_H
#define STATEMPCTROTTING_H

#include "quadruped_controller/FSM/FSMState.h"
#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/control/LegController.h"
#include "quadruped_controller/gait/Gait.h"
#include "quadruped_controller/mpc/CentroidalMPC.h"
#include "quadruped_controller/mpc/ComTrajectory.h"

namespace quadruped_controller
{

struct CtrlComponent;

class StateMPCTrotting : public FSMState
{
  public:
    explicit StateMPCTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component);

    void enter() override;
    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    void exit() override;
    FSMStateName checkChange() override;

  private:
    struct RawBaseState
    {
        Vec3 pos_world = Vec3::Zero();
        Vec3 linear_vel_world = Vec3::Zero();
        RotMat rotation_body_to_world = RotMat::Identity();
        Vec3 angular_vel_body = Vec3::Zero();
        Vec3 angular_vel_world = Vec3::Zero();
        Vec3 rpy_world = Vec3::Zero();
    };

    static constexpr int kMpcHorizonSteps = 16;
    static constexpr double kGaitPeriod = 1.0 / 3.0;
    static constexpr double kMpcTimeStep = kGaitPeriod / static_cast<double>(kMpcHorizonSteps);
    static constexpr double kMpcHorizonTime = kGaitPeriod;
    static constexpr double kMpcUpdatePeriod = kMpcTimeStep;
    static constexpr double kGaitDuty = 0.6;
    static constexpr double kSwingHeight = 0.10;
    static constexpr double kEntryHoldDuration = 1.0;
    static constexpr double kMaxNormalForce = 180.0;
    static constexpr double kFrictionCoefficient = 0.8;
    static constexpr double kDefaultBaseHeight = 0.27;
    static constexpr double kMinBaseHeight = 0.20;
    static constexpr double kMaxBaseHeight = 0.42;
    static constexpr double kMaxForwardVelocity = 0.50;
    static constexpr double kMaxLateralVelocity = 0.32;
    static constexpr double kMaxYawRate = 0.80;
    static constexpr double kMaxHeightRate = 0.08;
    static constexpr double kHipTorqueLimit = 23.0;
    static constexpr double kThighTorqueLimit = 23.0;
    static constexpr double kCalfTorqueLimit = 35.0;
    static constexpr double kHoldKp = 80.0;
    static constexpr double kHoldKd = 3.5;

    void captureHoldPosition();
    void captureNominalFootOffsets();
    bool readRawBaseState(RawBaseState &state) const;
    bool syncPinModelWithRawState(const RawBaseState &state);
    double gaitTime() const;
    void updateDesiredCommand(double dt, const RotMat &yaw_rotation_body_to_world);
    Vec34 buildCurrentFootLeversWorld(const RawBaseState &base_state) const;
    Vec34 buildTouchdownPositionsWorld(const RotMat &yaw_rotation_body_to_world, const Vec3 &desired_velocity_world,
                                       double desired_yaw_rate, const RawBaseState &base_state) const;
    Vec12 buildInitialMpcState(const RawBaseState &base_state) const;
    Vec34 buildFallbackContactForces(const VecInt4 &mask) const;
    Vec34 sanitizeContactForces(const Vec34 &forces_world, const VecInt4 &mask) const;
    Vec34 enforceCurrentContactMask(const Vec34 &forces_world, const VecInt4 &mask) const;
    static Mat3 bodyInertiaWorld(const RotMat &base_rotation_body_to_world);
    static double clampJointTorque(double torque, int joint);
    void commandHoldPosition();
    void commandTorqueControlDefaults();
    void writeJointTorques(const Vec12 &joint_torques);
    void commandZero();

    CtrlComponent &ctrl_component_;
    Gait gait_;
    ComTrajectory com_trajectory_;
    CentroidalMPC mpc_;
    LegController leg_controller_;
    Vec12 hold_joint_pos_ = Vec12::Zero();
    Vec34 nominal_foot_offsets_body_ = Vec34::Zero();
    Vec34 last_contact_forces_world_ = Vec34::Zero();
    Vec3 desired_base_pos_world_ = Vec3::Zero();
    double elapsed_time_{0.0};
    double next_debug_time_{0.0};
    double next_warning_time_{0.0};
    double next_mpc_update_time_{0.0};
    bool has_mpc_solution_{false};
};

} // namespace quadruped_controller

#endif // STATEMPCTROTTING_H
