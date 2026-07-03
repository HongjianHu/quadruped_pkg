#ifndef QUADRUPED_CONTROLLER__CONTROL__LEG_CONTROLLER_H
#define QUADRUPED_CONTROLLER__CONTROL__LEG_CONTROLLER_H

#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/gait/Gait.h"
#include "quadruped_controller/robot/go2_robot_data/PinGo2Model.h"
#include <array>
#include <string>

namespace quadruped_controller
{

struct LegOutput
{
    Vec3 tau = Vec3::Zero();
    Vec3 pos_des = Vec3::Zero();
    Vec3 pos_now = Vec3::Zero();
    Vec3 vel_des = Vec3::Zero();
    Vec3 vel_now = Vec3::Zero();
};

class LegController
{
  public:
    LegController();
    void setTouchdownPositionWorld(int leg, const Vec3 &position);
    void setTouchdownPositionsWorld(const Vec34 &positions);
    static int legIndex(const std::string &leg_name);

    LegOutput computeLegTorque(int leg, go2_robot_data::PinGo2Model &go2, Gait &gait, const Vec3 &contact_force_world,
                               double current_time);

    LegOutput computeLegTorque(const std::string &leg_name, go2_robot_data::PinGo2Model &go2, Gait &gait,
                               const Vec3 &contact_force_world, double current_time);

  private:
    static Mat3 swingKp();
    static Mat3 swingKd();

    VecInt4 last_mask_ = VecInt4::Constant(2);

    std::array<double, 4> takeoff_time_{};
    std::array<Vec3, 4> takeoff_pos_world_{};
    std::array<Vec3, 4> touchdown_pos_world_{};
    std::array<bool, 4> has_swing_plan_{};
    std::array<bool, 4> has_touchdown_target_{};
};

} // namespace quadruped_controller

#endif // QUADRUPED_CONTROLLER__CONTROL__LEG_CONTROLLER_H
