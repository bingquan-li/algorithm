#include "cost_Inflation.hh"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace algorithms
{

uint8_t
computeInflationCost(double d, CostInflationOptions const& o)
{
  // 约定 d >= 0，或为 +infinity（没有障碍物）
  if(d <= o.inscribed_radius)
  {
    return o.max_cost;
  }

  if(d >= o.inflation_radius)
  {
    return o.min_cost;
  }

  // 将距离归一化到 [0, 1]
  double const t =
      (d - o.inscribed_radius) / (o.inflation_radius - o.inscribed_radius);

  double factor = 0.0;

  switch(o.decay)
  {
    case DecayType::Linear:
      factor = 1.0 - t;
      break;

    case DecayType::Exponential:
      {
        double const e = std::exp(-o.decay_rate);
        factor = (std::exp(-o.decay_rate * t) - e) / (1.0 - e);
        break;
      }

    case DecayType::Gaussian:
      {
        double const e = std::exp(-o.decay_rate);
        factor = (std::exp(-o.decay_rate * t * t) - e) / (1.0 - e);
        break;
      }
  }

  double const cost = o.min_cost + (o.max_cost - o.min_cost) * factor;

  return static_cast<uint8_t>(
      std::lround(std::clamp(cost, double(o.min_cost), double(o.max_cost))));
}
} // namespace algorithms
