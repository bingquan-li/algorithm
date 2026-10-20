#include "later_boundary_visualization.hh"
#include "rerun/debug.hh"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace lateral_boundary_viz
{
namespace
{
using algorithms::Debug;
using Points = std::vector<Eigen::Vector2d>;
using Radius = Debug::Radius;

Points
path(local_escape::Candidate const& candidate)
{
  Points points;
  for(auto const& pose : candidate.poses)
    points.push_back(pose.position);
  return points;
}
void
polygon(std::string const& entity,
        local_escape::Pose const& pose,
        std::span<Eigen::Vector2d const> outline,
        Debug::Color color,
        float radius,
        float z)
{
  Debug::line(entity,
              local_escape::world_footprint(pose, outline),
              color,
              Radius::ui_points(radius),
              z,
              true);
}
} // namespace

void
begin(algorithms::GridMap<std::uint8_t> const& grid,
      local_escape::Pose const& start,
      std::span<Eigen::Vector2d const> outline)
{
  if(!Debug::active())
    return;
  Debug::clear("world");
  Debug::clear("planner_details");
  Debug::clear("planner_summary");
  auto const& g = grid.geometry();
  std::array<std::vector<Eigen::Vector3d>, 256> cells;
  for(std::size_t y = 0; y < g.height; ++y)
    for(std::size_t x = 0; x < g.width; ++x)
      cells[grid(x, y)].emplace_back(g.origin_x + (x + 0.5) * g.resolution,
                                     g.origin_y + (y + 0.5) * g.resolution,
                                     -0.001);
  std::array<Debug::Color, 4> const palette{
      {{241, 245, 249}, {96, 165, 250}, {38, 52, 69}, {245, 158, 11}}};
  for(std::size_t i = 0; i < cells.size(); ++i)
    if(!cells[i].empty())
      Debug::boxes("world/grid_map/class_" + std::to_string(i),
                   cells[i],
                   {g.resolution * 0.5, g.resolution * 0.5, 0.001},
                   i < palette.size() ? palette[i]
                                      : Debug::Color{120, 80, 160});
  polygon("world/robot/footprint", start, outline, {225, 29, 72}, 2.5F, 0.01F);
  Debug::point("world/robot/pose",
               start.position,
               {225, 29, 72},
               Radius::ui_points(5));
  Debug::arrow("world/robot/heading",
               start.position,
               {0.3 * std::cos(start.yaw), 0.3 * std::sin(start.yaw)},
               {225, 29, 72},
               0.006F);
  Debug::document(
      "legend",
      "World XY in meters; original cells grouped by class ID, no inflation.\n"
      "Red: robot. Yellow: centerline. Green/blue: left/right boundary.\n"
      "Widths end at the first occupied cell or map edge; body half-width is "
      "not subtracted.\n"
      "Boundary polylines connect samples and are not obstacle contours.\n"
      "Enable point labels to inspect index and distances.");
}

void
boundary(Eigen::Vector2d const& pose,
         Eigen::Vector2d const& forward,
         std::span<double const> samples,
         LateralBoundary const& widths)
{
  if(!Debug::active())
    return;
  Debug::clear("world/centerline");
  Debug::clear("world/boundary");
  Eigen::Vector2d const left(-forward.y(), forward.x());
  std::array<Points, 3> points;
  std::array<std::string, 3> const roots{"world/centerline",
                                         "world/boundary/left",
                                         "world/boundary/right"};
  std::array<Debug::Color, 3> const colors{
      {{245, 180, 0}, {22, 163, 74}, {37, 99, 235}}};
  for(std::size_t i = 0; i < samples.size(); ++i)
  {
    Eigen::Vector2d const center = pose + samples[i] * forward;
    points[0].push_back(center);
    points[1].push_back(center + widths.left_width[i] * left);
    points[2].push_back(center - widths.right_width[i] * left);
    std::array<double, 3> const values{samples[i],
                                       widths.left_width[i],
                                       widths.right_width[i]};
    for(std::size_t j = 0; j < 3; ++j)
    {
      std::ostringstream label;
      label << i << ": " << std::fixed << std::setprecision(3) << values[j]
            << " m";
      Debug::point(roots[j] + "/points/" + std::to_string(i),
                   points[j].back(),
                   colors[j],
                   Radius::ui_points(4),
                   0.02F,
                   label.str());
      if(j > 0)
      {
        Points const ray{center, points[j].back()};
        Debug::line(roots[j] + "/rays/" + std::to_string(i),
                    ray,
                    colors[j],
                    Radius::ui_points(0.75F));
      }
    }
  }
  for(std::size_t j = 0; j < 3; ++j)
    Debug::line(roots[j] + "/line",
                points[j],
                colors[j],
                Radius::ui_points(1.5F));
}

void
candidate(std::size_t index,
          local_escape::Candidate const& value,
          std::span<Eigen::Vector2d const> outline)
{
  if(!Debug::active())
    return;
  std::string const root = "world/planner/candidates/" + std::to_string(index);
  Debug::line(root + "/path",
              path(value),
              value.feasible ? Debug::Colors::purple.with_alpha(130)
                             : Debug::Color{120, 120, 120, 70},
              Radius::ui_points(0.7F),
              0.035F);
  if(value.rejected_pose)
  {
    Debug::point(root + "/rejected_pose",
                 value.rejected_pose->position,
                 Debug::Colors::red,
                 Radius::ui_points(2.5F));
    polygon(root + "/rejected_footprint",
            *value.rejected_pose,
            outline,
            Debug::Colors::red.with_alpha(70),
            0.5F,
            0.04F);
  }
  std::ostringstream detail;
  detail << value.reason << "\nnet reverse progress: " << value.reverse_progress
         << " m\nclearance lower bound: " << value.min_clearance
         << " m\nscore: " << value.score;
  Debug::document("planner_details/candidate_" + std::to_string(index),
                  detail.str());
}

void
finish(local_escape::Result const& result,
       std::span<Eigen::Vector2d const> outline)
{
  if(!Debug::active())
    return;
  std::ostringstream summary;
  summary << "Status: "
          << (result.status == local_escape::Status::success ? "success"
              : result.status == local_escape::Status::start_blocked
                  ? "start_blocked"
                  : "no_path")
          << "\nCandidates: " << result.candidates.size()
          << "\nBoundary preferred curvature: "
          << result.boundary.preferred_curvature
          << " 1/m\nLeft/right clearance slopes: " << result.boundary.left_slope
          << ", " << result.boundary.right_slope;
  if(result.selected)
  {
    auto const& selected = result.candidates[*result.selected];
    summary << "\nSelected: " << *result.selected
            << "\nNet reverse progress: " << selected.reverse_progress
            << " m\nConservative clearance: " << selected.min_clearance
            << " m\n";
    for(auto const& motion : selected.motions)
      summary << "distance=" << motion.distance
              << " m, curvature=" << motion.curvature
              << " 1/m, rotation=" << motion.rotation << " rad\n";
    Debug::line("world/planner/selected/path",
                path(selected),
                Debug::Colors::pink,
                Radius::ui_points(3),
                0.06F);
    std::size_t const stride =
        std::max<std::size_t>(1, (selected.poses.size() + 15) / 16);
    for(std::size_t i = 0; i < selected.poses.size(); i += stride)
      polygon("world/planner/selected/footprints/" + std::to_string(i),
              selected.poses[i],
              outline,
              Debug::Colors::pink.with_alpha(130),
              1.0F,
              0.05F);
    polygon("world/planner/selected/final_footprint",
            selected.poses.back(),
            outline,
            Debug::Colors::pink,
            2.0F,
            0.05F);
    for(std::size_t i = 0; i < selected.poses.size(); ++i)
    {
      auto context = *Debug::context();
      context.step = static_cast<std::int64_t>(i);
      Debug::ScopedContext const sample(context);
      polygon("world/planner/selected/animated_footprint",
              selected.poses[i],
              outline,
              {16, 185, 129},
              2.5F,
              0.07F);
    }
  }
  summary << "\nPurple: selected path. Green: animated footprint. Gray: "
             "rejected/prefix. Violet: feasible.\n"
             "Red: first rejected pose (collision OR safety margin). No "
             "selection means stop.\n"
             "Search is bounded. The step timeline is geometric sampling, not "
             "execution time.";
  Debug::document("planner_summary", summary.str());
}
} // namespace lateral_boundary_viz
