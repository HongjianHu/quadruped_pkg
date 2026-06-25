#ifndef FOOTENDCTRL_H
#define FOOTENDCTRL_H
#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/control/CtrlComponent.h"
#include <memory>
namespace quadruped_controller {
class KalmanFilterEstimate;
struct CtrlComponent;
class QuadrupedRobot;

class FeetEndCalc {
public:
  explicit FeetEndCalc(CtrlComponent &ctrl_component);

  void init();

  ~FeetEndCalc() = default;

  Vec3 calcFootPos(int index, Vec2 vxy_goal_global, double d_yaw_global,
                   double phase);

private:
  CtrlComponent &ctrl_component_;
  std::shared_ptr<QuadrupedRobot> &robot_model_;
  std::shared_ptr<KalmanFilterEstimate> &estimator_;

  Vec4 feet_radius_, feet_init_angle_;

  double k_x_, k_y_, k_yaw_;
  double t_stance_{}, t_swing_{};
};
} // namespace quadruped_controller
#endif // FOOTENDCTRL_H