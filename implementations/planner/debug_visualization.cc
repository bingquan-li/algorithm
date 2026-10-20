#include "debug_visualization.hh"
#include "rerun/debug.hh"
#include <algorithm>
#include <cmath>
#include <string>

namespace algorithms::planner_debug
{
namespace
{
std::vector<Eigen::Vector2d>
transformed(Pose const& pose, std::span<Eigen::Vector2d const> outline)
{
  Eigen::Rotation2Dd const rotation(pose.z());
  std::vector<Eigen::Vector2d> points;
  points.reserve(outline.size());
  for(auto const& p : outline)
  {
    points.emplace_back(pose.head<2>() + rotation * p);
  }
  return points;
}
} // namespace

void
begin(GridMap<std::uint8_t> const& grid,
      Pose const& start,
      std::span<Eigen::Vector2d const> outline,
      std::uint8_t occupied_threshold)
{
  if(!Debug::active())
  {
    return;
  }
  // A new frame must not inherit old candidates/selection if it exits early.
  Debug::clear("gridmap");
  Debug::text("gridmap/status", "planning");
  auto const& g = grid.geometry();
  std::vector<Eigen::Vector3d> centers;
  for(std::size_t y = 0; y < g.height; ++y)
  {
    for(std::size_t x = 0; x < g.width; ++x)
    {
      if(grid(x, y) >= occupied_threshold)
      {
        centers.emplace_back(g.origin_x + (x + 0.5) * g.resolution,
                             g.origin_y + (y + 0.5) * g.resolution,
                             0.15);
      }
    }
  }
  Debug::boxes("gridmap",
               centers,
               {g.resolution * 0.5, g.resolution * 0.5, 0.15},
               Debug::Colors::obstacle);
  auto const polygon = transformed(start, outline);
  Debug::line("gridmap/footprint/outline",
              polygon,
              Debug::Colors::footprint,
              0.008F,
              0.02F,
              true);
  double low = outline.front().x();
  double high = low;
  for(auto const& p : outline)
  {
    low = std::min(low, p.x());
    high = std::max(high, p.x());
  }
  Eigen::Vector2d const forward(std::cos(start.z()), std::sin(start.z()));
  std::vector<Eigen::Vector2d> axis{start.head<2>() + low * forward,
                                    start.head<2>() + high * forward};
  Debug::line("gridmap/footprint/longitudinal_axis",
              axis,
              Debug::Colors::reference);
  Debug::arrow("gridmap/footprint/forward",
               start.head<2>(),
               forward * std::max(0.1, (high - low) * 0.45),
               {220, 38, 38},
               0.025F,
               0.0F,
               "Forward",
               true);
}
void
boundary(BoundaryProfile const& boundary)
{
  if(!Debug::active())
  {
    return;
  }
  Debug::line("gridmap/boundary/left",
              boundary.boundary.left_points,
              {255, 0, 0});
  Debug::line("gridmap/boundary/right",
              boundary.boundary.right_points,
              {255, 0, 0});
}
void
rejection(std::string_view root,
          Pose const& pose,
          std::span<Eigen::Vector2d const> outline,
          double clearance,
          double half_motion_bound,
          double safety_margin)
{
  if(!Debug::active())
  {
    return;
  }
  std::string const path(root);
  auto const polygon = transformed(pose, outline);
  Debug::line(path + "/rejected_footprint",
              polygon,
              Debug::Colors::collision,
              0.008F,
              0.02F,
              true);
  Debug::point(path + "/rejected_pose",
               pose.head<2>(),
               Debug::Colors::collision,
               0.02F,
               0.05F,
               "collision_or_margin");
  Debug::scalar(path + "/clearance", clearance);
  Debug::scalar(path + "/half_motion_bound", half_motion_bound);
  Debug::scalar(path + "/clearance_lower_bound",
                std::max(0.0, clearance - half_motion_bound));
  Debug::scalar(path + "/safety_margin", safety_margin);
}
void
candidate(std::size_t index, Candidate const& value, bool selected)
{
  if(!Debug::active())
  {
    return;
  }
  Debug::Color color = Debug::Colors::pending;
  if(selected)
  {
    color = Debug::Colors::selected;
  }
  else if(value.feasible)
  {
    color = Debug::Colors::feasible;
  }
  else if(value.rejected_pose)
  {
    color = Debug::Colors::rejected;
  }
  std::string const root =
      "gridmap/candidates/candidate_" + std::to_string(index);
  std::vector<Eigen::Vector2d> points;
  points.reserve(value.poses.size());
  for(auto const& pose : value.poses)
  {
    points.emplace_back(pose.head<2>());
  }
  Debug::line(root + "/path",
              points,
              color,
              selected ? 0.006F : 0.004F,
              selected ? 0.04F : 0.025F);
  if(!value.poses.empty())
  {
    auto const& end = value.poses.back();
    std::string const label =
        "#" + std::to_string(index) + (selected ? " selected " : " ") +
        value.reason + " score=" + std::to_string(value.score) +
        " reverse=" + std::to_string(value.reverse_progress) + " m";
    Debug::point(root + "/endpoint",
                 end.head<2>(),
                 color,
                 0.012F,
                 0.04F,
                 label);
    Debug::arrow(root + "/heading",
                 end.head<2>(),
                 {0.12 * std::cos(end.z()), 0.12 * std::sin(end.z())},
                 color);
  }
}
void
finish(Result const& result)
{
  if(!Debug::active())
  {
    return;
  }

  std::string status = "no_path";
  if(result.status == Status::success)
  {
    status = "success";
  }
  else if(result.status == Status::start_blocked)
  {
    status = "start_blocked";
  }

  Debug::text("gridmap/status", status);
  if(!result.selected)
  {
    return;
  }
  auto const& selected = result.candidates[*result.selected];
  candidate(*result.selected, selected, true);
  std::vector<Eigen::Vector2d> points;
  points.reserve(selected.poses.size());
  for(auto const& pose : selected.poses)
  {
    points.emplace_back(pose.head<2>());
  }
  Debug::line("gridmap/selected_path", points, Debug::Colors::selected);
}
} // namespace algorithms::planner_debug
