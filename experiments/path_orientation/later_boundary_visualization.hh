#pragma once
#include "local_escape_planner.hh"

namespace lateral_boundary_viz
{
void
begin(algorithms::GridMap<std::uint8_t> const& grid,
      local_escape::Pose const& start,
      std::span<Eigen::Vector2d const> outline);
void
boundary(Eigen::Vector2d const& pose,
         Eigen::Vector2d const& forward,
         std::span<double const> samples,
         LateralBoundary const& widths);
void
candidate(std::size_t index,
          local_escape::Candidate const& value,
          std::span<Eigen::Vector2d const> outline);
void
finish(local_escape::Result const& result,
       std::span<Eigen::Vector2d const> outline);
} // namespace lateral_boundary_viz
