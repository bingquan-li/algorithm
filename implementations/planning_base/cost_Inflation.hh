// operations/cost_inflation.hh

#pragma once

#include <cstdint>
#include "grid_map.hh"

namespace algorithms
{

enum class DecayType : uint8_t
{
  Linear,
  Exponential,
  Gaussian
};

struct CostInflationOptions
{
  // 最大膨胀距离，单位 m
  double inflation_radius = 1.0;

  // 安全区域半径，内部使用最大代价
  double inscribed_radius = 0.2;

  // 衰减方式
  DecayType decay = DecayType::Exponential;

  // 衰减强度
  double decay_rate = 3.0;

  // 代价值
  uint8_t max_cost = 255;
  uint8_t min_cost = 0;
};

GridMap<uint8_t>
inflateCost(GridMap<float> const& distance_field,
            CostInflationOptions const& options);

} // namespace algorithms