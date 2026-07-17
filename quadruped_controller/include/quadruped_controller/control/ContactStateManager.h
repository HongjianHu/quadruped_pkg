#ifndef QUADRUPED_CONTROLLER__CONTROL__CONTACT_STATE_MANAGER_H
#define QUADRUPED_CONTROLLER__CONTROL__CONTACT_STATE_MANAGER_H

#include "quadruped_controller/common/mathTypes.h"

#include <array>

namespace quadruped_controller
{

enum class FootContactState
{
    SWING = 0,
    TOUCHDOWN_PENDING = 1,
    STANCE = 2,
    LIFTOFF = 3
};

struct ContactStateConfig
{
    double contact_force_enter_threshold = 8.0;
    double contact_force_exit_threshold = 4.0;
    double force_filter_time_constant = 0.01;
    double touchdown_confirm_duration = 0.006;
    double contact_loss_confirm_duration = 0.012;
    double touchdown_force_ramp_duration = 0.03;
    // 平地计划触地超过该时间仍无力反馈时，临时加入硬约束主动压脚；旧支撑仍不会释放。
    double touchdown_assumption_duration = 0.025;
    double assumed_contact_grace_duration = 0.08;
    double liftoff_force_ramp_duration = 0.025;
    double liftoff_force_release_duration = 0.006;
    // 新支撑腿尚未确认时，旧支撑腿至少保留该比例的承载能力，避免支撑集合瞬间变空。
    double support_transfer_min_activation = 0.60;
    int minimum_confirmed_support_legs = 2;
    double max_unconfirmed_support_duration = 0.10;
    double max_update_dt = 0.01;
};

struct ContactStateOutput
{
    VecInt4 hard_contact = VecInt4::Zero();
    Vec4 activation = Vec4::Zero();
    Vec4 filtered_normal_force = Vec4::Zero();
    VecInt4 force_confirmed = VecInt4::Zero();
    bool healthy = true;
};

// 将步态计划与足端力融合为带过渡的每腿接触状态。
// 该类不依赖ROS、FSM或WBC，因此可以独立测试。
class ContactStateManager
{
  public:
    explicit ContactStateManager(const ContactStateConfig &config = ContactStateConfig{});

    void reset();

    ContactStateOutput update(const VecInt4 &planned_contact, const Vec4 &raw_normal_force, double dt);

    FootContactState state(int leg) const;
    double stateElapsedTime(int leg) const;

    static const char *stateName(FootContactState state);

  private:
    void transitionTo(int leg, FootContactState next_state);
    static double sanitizeNormalForce(double force);

    ContactStateConfig config_;
    std::array<FootContactState, 4> states_{};
    Vec4 state_elapsed_time_ = Vec4::Zero();
    Vec4 contact_confirmation_time_ = Vec4::Zero();
    Vec4 contact_loss_time_ = Vec4::Zero();
    Vec4 activation_ = Vec4::Zero();
    Vec4 filtered_normal_force_ = Vec4::Zero();
    VecInt4 force_confirmed_ = VecInt4::Zero();
    bool filter_initialized_{false};
    double unsupported_time_{0.0};
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER__CONTROL__CONTACT_STATE_MANAGER_H
