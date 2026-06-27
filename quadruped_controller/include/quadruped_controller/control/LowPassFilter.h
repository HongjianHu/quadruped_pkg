#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

namespace quadruped_controller
{

class LowPassFilter
{
  public:
    LowPassFilter(double samplePeriod, double cutFrequency);

    ~LowPassFilter() = default;

    void addValue(double newValue);

    [[nodiscard]] double getValue() const;

    void clear();

  private:
    double weight_;
    double pass_value_{};
    bool start_;
};

} // namespace quadruped_controller

#endif // LOWPASSFILTER_H
