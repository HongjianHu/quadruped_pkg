#ifndef QUADRUPED_CONTROLLER__GAIT_H
#define QUADRUPED_CONTROLLER__GAIT_H

#include "quadruped_controller/common/mathTypes.h"
#include <Eigen/Dense>
#include <array>

namespace quadruped_controller
{

class Gait
{
  public:
    using ContactTable = Eigen::Matrix<int, 4, Eigen::Dynamic>;

    struct SwingTrajectoryPoint
    {
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        Vec3 acceleration = Vec3::Zero();
    };

    struct TouchdownInput
    {
        Vec3 base_pos_world = Vec3::Zero();
        Vec3 com_pos_world = Vec3::Zero();
        Vec3 com_vel_world = Vec3::Zero();
        RotMat yaw_rotation_body_to_world = RotMat::Identity();
        Vec3 hip_offset_body = Vec3::Zero();
        Vec3 desired_velocity_world = Vec3::Zero();
        Vec3 desired_position_world = Vec3::Zero();
        double yaw_rate_des_world = 0.0;
    };

    explicit Gait(double frequency_hz, double duty);
    Gait(double frequency_hz, double duty, const Vec4 &phase_offset);

    static Vec4 defaultPhaseOffset();

    VecInt4 computeCurrentMask(double time) const;
    ContactTable computeContactTable(double t0, double dt, int horizon) const;

    Vec4 computePhase(double time) const;

    void setSwingHeight(double swing_height);
    double swingHeight() const
    {
        return swing_height_;
    }

    void resetSwingState(const Vec34 &foot_positions);

    void updateSwingTrajectory(double time, const Vec34 &current_foot_positions, const Vec34 &touchdown_positions,
                               Vec34 &target_positions, Vec34 &target_velocities, Vec34 &target_accelerations);

    Vec3 computeTouchdownWorld(const TouchdownInput &input) const;

    Vec34 computeTouchdownWorlds(const TouchdownInput &input, const Vec34 &hip_offsets_body) const;

    static double defaultSwingHeight();

    static SwingTrajectoryPoint evaluateSwingTrajectory(const Vec3 &start_pos, const Vec3 &end_pos, double time,
                                                        double swing_time);

    static SwingTrajectoryPoint evaluateSwingTrajectory(const Vec3 &start_pos, const Vec3 &end_pos, double time,
                                                        double swing_time, double swing_height);
    double duty() const
    {
        return gait_duty_;
    }
    double frequency() const
    {
        return gait_hz_;
    }
    double period() const
    {
        return gait_period_;
    }
    double stanceTime() const
    {
        return stance_time_;
    }
    double swingTime() const
    {
        return swing_time_;
    }
    const Vec4 &phaseOffset() const
    {
        return phase_offset_;
    }

  private:
    struct SwingState
    {
        bool active = false;
        Vec3 start_pos = Vec3::Zero();
        Vec3 end_pos = Vec3::Zero();
    };

    double gait_duty_{};
    double gait_hz_{};
    double gait_period_{};
    double stance_time_{};
    double swing_time_{};
    double swing_height_{0.1};
    std::array<SwingState, 4> swing_states_{};
    Vec4 phase_offset_{};
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER__GAIT_H