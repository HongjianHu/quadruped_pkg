#ifndef STATEMPCWBCTROTTING_H
#define STATEMPCWBCTROTTING_H

#include "quadruped_controller/FSM/FSMState.h"
#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/control/LegController.h"
#include "quadruped_controller/control/WbcController.h"
#include "quadruped_controller/gait/Gait.h"
#include "quadruped_controller/mpc/CentroidalMPC.h"
#include "quadruped_controller/mpc/ComTrajectory.h"

namespace quadruped_controller
{

struct CtrlComponent;

class StateMPCWBCTrotting : public FSMState
{
  public:
    explicit StateMPCWBCTrotting(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component);

    void enter() override;
    void run(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    void exit() override;
    FSMStateName checkChange() override;

  private:
    enum class TorqueAuthority
    {
        FALLBACK,
        BLENDING_TO_WBC,
        WBC
    };

    struct RawBaseState
    {
        Vec3 pos_world = Vec3::Zero();
        Vec3 linear_vel_world = Vec3::Zero();
        RotMat rotation_body_to_world = RotMat::Identity();
        Vec3 angular_vel_body = Vec3::Zero();
        Vec3 angular_vel_world = Vec3::Zero();
        Vec3 rpy_world = Vec3::Zero();
    };

    struct FootTrajectoryReference
    {
        Vec34 position_world = Vec34::Zero();
        Vec34 velocity_world = Vec34::Zero();
        Vec34 acceleration_world = Vec34::Zero();
    };

    struct JointImpedanceReference
    {
        Vec12 position = Vec12::Zero();
        Vec12 velocity = Vec12::Zero();
    };

    static constexpr int kMpcHorizonSteps = 16;
    static constexpr double kGaitPeriod = 1.0 / 3.0;
    static constexpr double kMpcTimeStep = kGaitPeriod / static_cast<double>(kMpcHorizonSteps);
    static constexpr double kMpcHorizonTime = kGaitPeriod;
    static constexpr double kMpcUpdatePeriod = kMpcTimeStep;
    static constexpr double kGaitDuty = 0.6;
    static constexpr double kSwingHeight = 0.10;
    static constexpr double kEntryHoldDuration = 1.0;
    static constexpr double kDefaultBaseHeight = 0.27;
    static constexpr double kMinBaseHeight = 0.20;
    static constexpr double kMaxBaseHeight = 0.42;
    static constexpr double kMaxForwardVelocity = 1.00;
    static constexpr double kMaxLateralVelocity = 0.32;
    static constexpr double kMaxYawRate = 0.80;
    static constexpr double kMaxHeightRate = 0.08;
    static constexpr double kHipTorqueLimit = 23.0;
    static constexpr double kThighTorqueLimit = 23.0;
    static constexpr double kCalfTorqueLimit = 35.0;
    static constexpr double kHoldKp = 80.0;
    static constexpr double kHoldKd = 3.5;
    static constexpr double kBasePositionKp = 60.0;
    static constexpr double kBaseLinearVelocityKd = 10.0;
    static constexpr double kBaseOrientationKp = 80.0;
    static constexpr double kBaseAngularVelocityKd = 10.0;
    static constexpr double kSwingPositionKp = 400.0;
    static constexpr double kSwingVelocityKd = 75.0;
    static constexpr double kSwingJointKp = 1.0;
    static constexpr double kSwingJointKd = 0.1;
    static constexpr double kStanceJointKp = 0.0;
    static constexpr double kStanceJointKd = 0.1;
    static constexpr double kJointVelocityInverseDamping = 1.0e-4;
    static constexpr double kJointVelocityKd = 2.0;
    static constexpr double kMaxBaseLinearAcceleration = 20.0;
    static constexpr double kMaxBaseAngularAcceleration = 30.0;
    static constexpr double kMaxSwingFootAcceleration = 120.0;
    static constexpr double kMaxJointAcceleration = 40.0;
    static constexpr std::size_t kRequiredWbcSuccessCycles = 250;
    static constexpr std::size_t kRequiredWbcContactSwitches = 2;
    static constexpr double kWbcBlendDuration = 1.0;
    static constexpr double kMaxTorqueRate = 1000.0;
    static constexpr double kMaxTorqueRateDt = 0.01;
    static constexpr double kMinWbcBaseHeight = 0.18;
    static constexpr double kMaxWbcBaseHeight = 0.48;
    static constexpr double kMaxWbcAbsRollPitch = 0.60;
    static constexpr double kActualAccelerationFilterTimeConstant = 0.03;

    void captureHoldPosition();
    void captureNominalFootOffsets();
    bool readRawBaseState(RawBaseState &state) const;
    bool syncPinModelWithRawState(const RawBaseState &state);

    double gaitTime() const;
    void updateSharedGaitState(double gait_time, const VecInt4 &contact);
    Vec4 readFootNormalForces() const;

    void updateDesiredCommand(double dt, const RotMat &yaw_rotation_body_to_world);

    Vec34 buildCurrentFootLeversWorld(const RawBaseState &base_state) const;
    Vec34 buildTouchdownPositionsWorld(const RotMat &yaw_rotation_body_to_world, const Vec3 &desired_velocity_world,
                                       double desired_yaw_rate, const RawBaseState &base_state) const;
    Vec12 buildInitialMpcState(const RawBaseState &base_state) const;
    Vec34 buildFallbackContactForces(const VecInt4 &mask) const;

    Vec34 sanitizeContactForces(const Vec34 &forces_world, const VecInt4 &mask) const;
    Vec34 enforceCurrentContactMask(const Vec34 &forces_world, const VecInt4 &mask) const;

    WbcInput buildWbcInput(const RawBaseState &base_state, const VecInt4 &contact,
                           const Vec34 &desired_swing_foot_acceleration_world);
    Vec6 buildDesiredBaseAccelerationTangent(const RawBaseState &base_state) const;
    Vec12 buildDesiredJointAcceleration() const;
    FootTrajectoryReference buildFootTrajectoryReference(double gait_time, const VecInt4 &contact,
                                                         const Vec34 &touchdown_positions_world);
    JointImpedanceReference buildJointImpedanceReference(const RawBaseState &base_state,
                                                         const FootTrajectoryReference &foot_reference) const;

    static Mat3 bodyInertiaWorld(const RotMat &base_rotation_body_to_world);
    static double jointTorqueLimit(int joint);
    static double clampJointTorque(double torque, int joint);
    bool isBaseStateSafeForWbc(const RawBaseState &base_state) const;
    Vec12 selectCommandedTorques(const Vec12 &fallback_torques, const WbcOutput &wbc_output, const VecInt4 &contact,
                                 bool base_state_safe, double dt);
    Vec12 applyTorqueRateLimit(const Vec12 &target_torques, double dt);
    void resetTorqueAuthority();
    const char *torqueAuthorityName() const;
    void commandHoldPosition();
    void commandHybridTorqueControl(const JointImpedanceReference &reference, const VecInt4 &contact,
                                    double gain_scale);
    void writeJointTorques(const Vec12 &joint_torques);
    void commandZero();

    CtrlComponent &ctrl_component_;
    Gait gait_;
    ComTrajectory com_trajectory_;
    CentroidalMPC mpc_;
    WbcController wbc_controller_;
    LegController fallback_leg_controller_;
    Vec12 hold_joint_pos_ = Vec12::Zero();
    Vec34 nominal_foot_offsets_body_ = Vec34::Zero();
    Vec34 last_contact_forces_world_ = Vec34::Zero();
    Vec3 desired_base_pos_world_ = Vec3::Zero();
    double elapsed_time_{0.0};
    double next_debug_time_{0.0};
    double next_warning_time_{0.0};
    double next_wbc_warning_time_{0.0};
    double next_mpc_update_time_{0.0};
    bool has_mpc_solution_{false};
    VecInt4 last_mpc_contact_ = VecInt4::Zero();
    WbcOutput last_wbc_output_;
    std::size_t wbc_solve_count_{0};
    std::size_t wbc_success_count_{0};
    std::size_t wbc_failure_count_{0};
    double last_wbc_solve_time_us_{0.0};
    double accumulated_wbc_solve_time_us_{0.0};
    TorqueAuthority torque_authority_{TorqueAuthority::FALLBACK};
    std::size_t consecutive_wbc_successes_{0};
    std::size_t qualified_contact_switches_{0};
    std::size_t wbc_fallback_event_count_{0};
    VecInt4 last_authority_contact_ = VecInt4::Ones();
    bool has_authority_contact_{false};
    Vec4 last_raw_foot_normal_forces_ = Vec4::Zero();
    VecInt4 last_wbc_contact_ = VecInt4::Zero();
    Vec6 last_desired_base_acceleration_tangent_ = Vec6::Zero();
    double last_base_vertical_velocity_{0.0};
    double filtered_actual_base_acceleration_z_{0.0};
    bool has_previous_base_vertical_velocity_{false};
    double wbc_blend_{0.0};
    Vec12 last_commanded_torques_ = Vec12::Zero();
};

} // namespace quadruped_controller

#endif // STATEMPCWBCTROTTING_H
