#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include <chrono>

#include "quadruped_controller/common/enumClass.h"
#include "quadruped_controller/common/mathTypes.h"

namespace quadruped_controller {

inline long long getSystemTime() {
  const auto now = std::chrono::system_clock::now();
  const auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(duration)
      .count();
}

class WaveGenerator {
public:
  /// @param period   步态周期 [s]
  /// @param st_ratio 支撑相占比 (0, 1)，如 0.6 表示 60% 触地
  /// @param bias     各腿相位偏置 [FR, FL, RR, RL]，范围 [0, 1]
  WaveGenerator(double period, double st_ratio, const Vec4 &bias);

  ~WaveGenerator() = default;

  /// 每控制周期调用一次，更新 phase 和 contact
  void update();

  double getTStance() const { return period_ * st_ratio_; }
  double getTSwing() const { return period_ * (1 - st_ratio_); }
  double getT() const { return period_; }

  Vec4 phase_;          // 各腿归一化相位 [0, 1]
  VecInt4 contact_;     // 接触状态 (1=支撑, 0=摆动)
  WaveStatus status_{}; // 当前波浪状态

private:
  /// 根据时间计算 phase, contact, status
  void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

  double period_;
  double st_ratio_;
  Vec4 bias_;

  Vec4 normal_t_; // 归一化时间 [0, 1)
  Vec4 phase_past_;
  VecInt4 contact_past_;
  VecInt4 switch_status_;
  WaveStatus status_past_;

  long long start_t_{};
};

} // namespace quadruped_controller

#endif // WAVEGENERATOR_H
