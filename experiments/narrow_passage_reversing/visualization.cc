#include "visualization.hh"
#include <stdexcept>
#include "timing.hh"
#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

namespace narrow_passage
{
namespace
{
template <typename IsOccupied>
void
log_grid_obstacles(rerun::RecordingStream const& rec,
                   std::string_view entity_path,
                   algorithms::GridMap<std::uint8_t> const& map,
                   IsOccupied is_occupied,
                   float display_height = 0.3F)
{
  auto const& g = map.geometry();

  std::vector<rerun::components::Translation3D> centers;
  std::vector<rerun::components::HalfSize3D> half_sizes;

  auto const half_cell = static_cast<float>(g.resolution * 0.5);
  float const half_height = display_height * 0.5F;

  for(std::size_t y = 0; y < g.height; ++y)
  {
    for(std::size_t x = 0; x < g.width; ++x)
    {
      if(!is_occupied(map(x, y)))
      {
        continue; // 自由区域不生成任何几何体。
      }

      auto const cx = static_cast<float>(
          g.origin_x + (static_cast<double>(x) + 0.5) * g.resolution);
      auto const cy = static_cast<float>(
          g.origin_y + (static_cast<double>(y) + 0.5) * g.resolution);

      centers.emplace_back(cx, cy, half_height);
      half_sizes.emplace_back(half_cell, half_cell, half_height);
    }
  }

  rec.log(entity_path,
          rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes)
              .with_colors({rerun::Rgba32(80, 175, 170)})
              .with_fill_mode(rerun::FillMode::Solid));
}

rerun::Position3D
position(Eigen::Vector2d const& point, float z = 0.02F)
{
  return {static_cast<float>(point.x()), static_cast<float>(point.y()), z};
}

void
log_line(rerun::RecordingStream const& rec,
         std::string const& name,
         std::vector<Eigen::Vector2d> const& points,
         rerun::Color color)
{
  std::vector<rerun::Position3D> positions;
  positions.reserve(points.size());
  for(auto const& point : points)
  {
    positions.push_back(position(point));
  }
  rec.log(name,
          rerun::LineStrips3D{rerun::LineStrip3D{positions}}
              .with_colors({color})
              .with_radii({0.008F}));
}


void
log_footprint(rerun::RecordingStream const& rec,
              std::string_view entity_path,
              algorithms::RobotGeometry const& geometry,
              algorithms::RearAxlePose const& pose)
{
  auto const footprint = geometry.footprint_at(pose);
  if(!footprint)
  {
    return;
  }

  using Point = Eigen::Vector2d;

  std::string const root(entity_path);



  Point const center = pose.pose.head<2>();
  Point const forward(std::cos(pose.pose.z()), std::sin(pose.pose.z()));

  // 1. 八边形轮廓
  auto outline = footprint->vertices();
  outline.push_back(outline.front());

  log_line(rec, root + "/outline", outline, rerun::Color(13, 148, 136));

  // 2. 车身纵向中心线
  auto const& extents = geometry.extents();
  Point const rear_end = center - extents.rear_m * forward;
  Point const front_end = center + extents.front_m * forward;

  log_line(rec,
           root + "/longitudinal_axis",
           {rear_end, front_end},
           rerun::Color(148, 163, 184));

  // 3. 前进方向
  double const length = extents.front_m + extents.rear_m;
  Point const arrow = forward * std::max(0.1, length * 0.45);

  rec.log(root + "/forward",
          rerun::Arrows3D::from_vectors({{static_cast<float>(arrow.x()),
                                          static_cast<float>(arrow.y()),
                                          0.0F}})
              .with_origins({rerun::Position3D{static_cast<float>(center.x()),
                                               static_cast<float>(center.y()),
                                               0.0}})
              .with_colors({rerun::Color(220, 38, 38)})
              .with_radii({0.025F})
              .with_labels({"Forward"})
              .with_show_labels(true));
}

// 每个候选独立成实体，便于在 Rerun 中单独显示/隐藏。
// poses 只含通过连续碰撞检查的部分；拒绝位姿单独标记，不接成安全路径。
void
log_candidates(rerun::RecordingStream const& rec,
               algorithms::Result const& result)
{
  for(std::size_t i = 0; i < result.candidates.size(); ++i)
  {
    auto const& candidate = result.candidates[i];
    bool const selected = result.selected && *result.selected == i;
    auto color = rerun::Color(245, 180, 45);
    if(selected)
    {
      color = rerun::Color(50, 220, 80);
    }
    else if(candidate.feasible)
    {
      color = rerun::Color(60, 150, 255);
    }
    else if(candidate.rejected_pose)
    {
      color = rerun::Color(240, 80, 80);
    }

    std::string const root =
        "gridmap/candidates/candidate_" + std::to_string(i);
    std::vector<rerun::Position3D> points;
    points.reserve(candidate.poses.size());
    for(auto const& pose : candidate.poses)
    {
      points.push_back(position(pose.head<2>(), selected ? 0.04F : 0.025F));
    }
    if(points.size() >= 2)
    {
      rec.log(root + "/path",
              rerun::LineStrips3D{rerun::LineStrip3D{points}}
                  .with_colors({color})
                  .with_radii({selected ? 0.006F : 0.004F}));
    }
    if(!candidate.poses.empty())
    {
      auto const& end = candidate.poses.back();
      std::string const label =
          "#" + std::to_string(i) + (selected ? " selected " : " ") +
          candidate.reason + " score=" + std::to_string(candidate.score) +
          " reverse=" + std::to_string(candidate.reverse_progress) + " m";
      rec.log(root + "/endpoint",
              rerun::Points3D({position(end.head<2>(), 0.04F)})
                  .with_colors({color})
                  .with_radii({0.012F})
                  .with_labels({label})
                  .with_show_labels(false));
      // 原地旋转没有中心位移，用末端朝向箭头显示其结果。
      rec.log(root + "/heading",
              rerun::Arrows3D::from_vectors(
                  {{static_cast<float>(0.12 * std::cos(end.z())),
                    static_cast<float>(0.12 * std::sin(end.z())),
                    0.0F}})
                  .with_origins({position(end.head<2>(), 0.04F)})
                  .with_colors({color})
                  .with_radii({0.004F}));
    }
    if(candidate.rejected_pose)
    {
      rec.log(
          root + "/rejected_pose",
          rerun::Points3D({position(candidate.rejected_pose->head<2>(), 0.05F)})
              .with_colors({rerun::Color(255, 40, 40)})
              .with_radii({0.02F})
              .with_labels({"#" + std::to_string(i) + " collision_or_margin"})
              .with_show_labels(false));
    }
  }
}


} // namespace

void
log_scene(rerun::RecordingStream const& rec,
          Scenario const& scenario,
          std::uint8_t occupied_threshold)
{
  timed_call("grid obstacles build/submit",
             [&]
             {
               log_grid_obstacles(rec,
                                  "gridmap",
                                  scenario.grid,
                                  [occupied_threshold](std::uint8_t value)
                                  { return value >= occupied_threshold; });
             });
  timed_call("footprint build/submit",
             [&]
             {
               log_footprint(rec,
                             "gridmap/footprint",
                             scenario.geometry,
                             scenario.start_pose);
             });
}

void
log_result(rerun::RecordingStream const& rec, algorithms::Result const& result)
{
  // 展示规划实际使用的初始边界，不另行计算不同范围的辅助边界。
  timed_call("boundary lines submit",
             [&]
             {
               log_line(rec,
                        "gridmap/boundary/left",
                        result.boundary.boundary.left_points,
                        rerun::Color(255, 0, 0));
               log_line(rec,
                        "gridmap/boundary/right",
                        result.boundary.boundary.right_points,
                        rerun::Color(255, 0, 0));
             });
  timed_call("all candidate paths build/submit",
             [&] { log_candidates(rec, result); });
  if(!result.selected)
  {
    return;
  }
  std::vector<Eigen::Vector2d> path;
  auto const& selected = result.candidates[*result.selected];
  path.reserve(selected.poses.size());
  for(auto const& pose : selected.poses)
  {
    path.emplace_back(pose.head<2>());
  }
  timed_call(
      "selected path submit",
      [&]
      {
        log_line(rec, "gridmap/selected_path", path, rerun::Color(50, 220, 80));
      });
}
void
visualize(Scenario const& scenario,
          algorithms::Result const& result,
          std::uint8_t occupied_threshold,
          std::optional<std::filesystem::path> const& blueprint)
{
  if(blueprint && !std::filesystem::is_regular_file(*blueprint))
  {
    throw std::runtime_error("Blueprint not found: " + blueprint->string());
  }
  rerun::RecordingStream rec("narrow_passage");
  timed_call("rerun.spawn", [&] { rec.spawn().exit_on_failure(); });
  if(blueprint)
  {
    timed_call("rerun.blueprint submit",
               [&] { rec.log_file_from_path(blueprint->string()); });
  }
  log_scene(rec, scenario, occupied_threshold);
  log_result(rec, result);
}
} // namespace narrow_passage
