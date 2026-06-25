#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "quadruped_controller/common/enumClass.h"
#include "quadruped_controller/common/mathTypes.h"
#include "quadruped_controller/control/CtrlComponent.h"
#include "quadruped_controller/gait/FeetEndCalc.h"
#include "quadruped_controller/gait/WaveGenerator.h"
#include <memory>
#include <utility>
namespace quadruped_controller {
class KalmanFilterEstimate;
class WaveGenerator;
struct CtrlComponent;

class GaitGenerator {
public:
  explicit GaitGenerator(CtrlComponent &ctrl_component);

  ~GaitGenerator() = default;

  void setGait(Vec2 vxy_goal_global, double d_yaw_goal, double gait_height);

  void generate(Vec34 &feet_pos, Vec34 &feet_vel);

  void restart();

private:
  Vec3 getFootPos(int i);

  Vec3 getFootVel(int i);

  static double cycloidXYPosition(double startXY, double endXY, double phase);

  static double cycloidZPosition(double startZ, double height, double phase);

  double cycloidXYVelocity(double startXY, double endXY, double phase) const;

  double cycloidZVelocity(double height, double phase) const;

  std::shared_ptr<WaveGenerator> &wave_generator_;
  std::shared_ptr<KalmanFilterEstimate> &estimator_;
  FeetEndCalc feet_end_calc_;

  double gait_height_{};
  Vec2 vxy_goal_;
  double d_yaw_goal_{};
  Vec34 start_p_, end_p_, ideal_p_, past_p_;
  bool first_run_;
};
} // namespace quadruped_controller
#endif // GAITGENERATOR_H
