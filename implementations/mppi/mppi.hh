#pragma once

#include "grid_map.hh"

#include <Eigen/Eigen>

namespace algorithms::mppi
{
struct Options
{};

struct Result
{};

struct RobotState
{};

class MppiPlanner
{
public:
  void
  reset();

  Result
  plan(RobotState const& state,
       std::vector<Eigen::Vector2d> const& reference_path,
       GridMap<std::uint8_t> const& grid_map);

private:
};

} // namespace algorithms::mppi