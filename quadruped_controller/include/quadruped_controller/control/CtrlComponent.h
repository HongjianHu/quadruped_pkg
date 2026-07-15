#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include "quadruped_controller/control/Estimator.h"
#include "quadruped_controller/robot/QuadrupedRobot.h"
#include <memory>

namespace quadruped_controller
{

class QuadrupedRobot;
class KalmanFilterEstimate;

struct CtrlComponent
{

    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<KalmanFilterEstimate> estimator_;
    VecInt4 gait_contact_ = VecInt4::Ones();
    Vec4 gait_phase_ = Vec4::Constant(0.5);

    CtrlComponent() = default;
};

} // namespace quadruped_controller

#endif
