#include "quadruped_controller/control/LowPassFilter.h"
#include <cmath>

namespace quadruped_controller
{

LowPassFilter::LowPassFilter(const double samplePeriod, const double cutFrequency)
{
    weight_ = 1.0 / (1.0 + 1.0 / (2.0 * M_PI * samplePeriod * cutFrequency));
    start_ = false;
}

void LowPassFilter::addValue(const double newValue)
{
    if (!start_)
    {
        start_ = true;
        pass_value_ = newValue;
    }
    pass_value_ = weight_ * newValue + (1 - weight_) * pass_value_;
}

double LowPassFilter::getValue() const
{
    return pass_value_;
}

void LowPassFilter::clear()
{
    start_ = false;
}

} // namespace quadruped_controller
