// operations/inflation.hh

#pragma once

#include "grid_map.hh"

namespace algorithms
{

enum class InflationShape : uint8_t
{
  Circle,
  Rectangle
};

struct InflationOptions
{
  double radius = 0.2; // 单位：米
  InflationShape shape = InflationShape::Circle;

  uint8_t obstacle_value = 255;
  uint8_t inflated_value = 255;
};

GridMap<uint8_t>
inflate(GridMap<uint8_t> const& input, InflationOptions const& options);

} // namespace algorithms