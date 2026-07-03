#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include "quadruped_controller/control/Estimator.h"
#include "quadruped_controller/gait/WaveGenerator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"
#include <memory>

namespace quadruped_controller
{

class QuadrupedRobot;
class KalmanFilterEstimate;
class WaveGenerator;

struct CtrlComponent
{

    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<KalmanFilterEstimate> estimator_;
    std::shared_ptr<WaveGenerator> wave_generator_;

    CtrlComponent() = default;
};

} // namespace quadruped_controller

#endif
