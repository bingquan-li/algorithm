#pragma once

#include <Eigen/Core>
#include <rerun.hpp>
#include <span>
#include <utility>
#include <vector>

namespace algorithms::viz
{
// Explicit conversion: no SDK CollectionAdapter specializations or borrowed buffers.
inline rerun::Position3D
position(Eigen::Vector2d const& point, float z = 0.02F)
{
  return {static_cast<float>(point.x()), static_cast<float>(point.y()), z};
}

inline rerun::LineStrip3D
line_xy(std::span<Eigen::Vector2d const> points, float z = 0.0F)
{
  std::vector<rerun::Vec3D> vertices;
  vertices.reserve(points.size());
  for(auto const& p : points)
  {
    vertices.emplace_back(static_cast<float>(p.x()),
                          static_cast<float>(p.y()),
                          z);
  }
  return {std::move(vertices)};
}

// XY paths may store heading in the third component: explicitly discard it.
inline rerun::LineStrip3D
line_xy(std::span<Eigen::Vector3d const> points, float z = 0.0F)
{
  std::vector<rerun::Vec3D> vertices;
  vertices.reserve(points.size());
  for(auto const& p : points)
  {
    vertices.emplace_back(static_cast<float>(p.x()),
                          static_cast<float>(p.y()),
                          z);
  }
  return {std::move(vertices)};
}

inline rerun::LineStrip3D
line_xyz(std::span<Eigen::Vector3d const> points)
{
  std::vector<rerun::Vec3D> vertices;
  vertices.reserve(points.size());
  for(auto const& p : points)
  {
    vertices.emplace_back(static_cast<float>(p.x()),
                          static_cast<float>(p.y()),
                          static_cast<float>(p.z()));
  }
  return {std::move(vertices)};
}
} // namespace algorithms::viz
