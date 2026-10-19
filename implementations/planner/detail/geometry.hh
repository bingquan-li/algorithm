#pragma once

#include "../local_escape.hh"
#include <utility>

namespace algorithms::local_escape_detail
{
inline constexpr double epsilon = 1e-10;
Eigen::Vector2d
direction(Pose const& pose);

std::pair<double, double>
lateral_extent(std::span<Eigen::Vector2d const> footprint, double x);

bool
inside(Eigen::Vector2d const& p, std::span<Eigen::Vector2d> polygon);

double
point_segment_distance(Eigen::Vector2d const& p,
                       Eigen::Vector2d const& a,
                       Eigen::Vector2d const& b);

double
cross(Eigen::Vector2d const& a, Eigen::Vector2d const& b);

bool
segments_intersect(Eigen::Vector2d const& a,
                   Eigen::Vector2d const& b,
                   Eigen::Vector2d const& c,
                   Eigen::Vector2d const& d);

double
cell_distance(std::span<Eigen::Vector2d> polygon,
              Eigen::Vector2d const& lo,
              Eigen::Vector2d const& hi);

std::vector<Eigen::Vector2d>
world_footprint(Pose const& pose, std::span<Eigen::Vector2d const> footprint);

Pose
advance(Pose const& start, Motion const& motion, double fraction);
} // namespace algorithms::local_escape_detail
