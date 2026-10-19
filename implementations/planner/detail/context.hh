#pragma once
#include "../local_escape.hh"


#include <algorithm>


namespace algorithms::local_escape_detail
{
// 只读借用，生命周期限定在一次 plan 调用中；结果与统计单独传递。
struct Context
{
  GridMap<std::uint8_t> const& grid;
  Pose const& start;
  std::span<Eigen::Vector2d const> footprint;
  Config const& config;
  double radius;
  double step_bound;

  Context(GridMap<std::uint8_t> const& grid,
          Pose const& start,
          std::span<Eigen::Vector2d const> footprint,
          Config const& config,
          double radius,
          double step_bound)
    : grid(grid)
    , start(start)
    , footprint(footprint)
    , config(config)
    , radius(radius)
    , step_bound(step_bound)
  {}

  Context(Context const&) = delete;
  Context&
  operator=(Context const&) = delete;
  Context(Context&&) = delete;
  Context&
  operator=(Context&&) = delete;
};

inline Context
make_context(GridMap<std::uint8_t> const& grid,
             Pose const& start,
             std::span<Eigen::Vector2d const> footprint,
             Config const& config)
{
  double radius = 0.0;
  for(auto const& p : footprint)
  {
    radius = std::max(radius, p.norm());
  }
  double const step_bound =
      std::min({config.max_corner_step,
                (config.clearance_cap - config.safety_margin) * 0.5});
  return {grid, start, footprint, config, radius, step_bound};
}
} // namespace algorithms::local_escape_detail
