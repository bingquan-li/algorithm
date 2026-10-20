#pragma once
#include <string_view>

#include "local_escape.hh"

namespace algorithms::planner_debug
{
// Planner-specific paths/styles live here; algorithms call these at the actual events.
void
begin(GridMap<std::uint8_t> const& grid,
      Pose const& start,
      std::span<Eigen::Vector2d const> outline,
      std::uint8_t occupied_threshold);
void
boundary(BoundaryProfile const& boundary);
void
rejection(std::string_view root,
          Pose const& pose,
          std::span<Eigen::Vector2d const> outline,
          double clearance,
          double half_motion_bound,
          double safety_margin);
void
candidate(std::size_t index, Candidate const& candidate, bool selected = false);
void
finish(Result const& result);
} // namespace algorithms::planner_debug
