#include "grid_map.hh"
#include "footprint.hh"
#include "rerun/rerun_wrapper.hh"
#include "lateral_boundary.hh"
#include "local_escape.hh"

#include <rerun.hpp>



#include <chrono>
#include <cstdio>
#include <utility>
#include <vector>
#include <cstdint>
#include <iostream>
#include <string>
#include <string_view>


namespace
{
// steady_clock 不受系统时间调整影响；异常退出作用域时也记录耗时。
class ScopedTimer
{
public:
  explicit ScopedTimer(char const* label)
    : label_(label)
    , start_(std::chrono::steady_clock::now())
  {}

  ~ScopedTimer()
  {
    double const milliseconds = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - start_)
                                    .count();
    std::fprintf(stderr, "[timing] %s: %.3f ms\n", label_, milliseconds);
  }

private:
  char const* label_;
  std::chrono::steady_clock::time_point start_;
};

template <typename Function>
decltype(auto)
timed_call(char const* label, Function&& function)
{
  ScopedTimer const timer(label);
  return std::forward<Function>(function)();
}
} // namespace


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
  rec.log(name,
          rerun::LineStrips3D{rerun::LineStrip3D{points}}
              .with_colors({color})
              .with_radii({0.008F}));
}


void
log_footprint(rerun::RecordingStream const& rec,
              std::string_view entity_path,
              algorithms::Footprint const& footprint)
{
  if(footprint.vertices.size() < 3)
  {
    return;
  }

  using Point = Eigen::Vector2d;

  std::string const root(entity_path);



  Point const& center = footprint.centroid;
  Point const forward = footprint.forward.normalized();

  // 1. 八边形轮廓
  auto outline = footprint.vertices;
  outline.push_back(outline.front());

  log_line(rec, root + "outline", outline, rerun::Color(13, 148, 136));

  // 2. 车身纵向中心线
  Point const rear_end = center - footprint.rear_extent * forward;
  Point const front_end = center + footprint.front_extent * forward;

  log_line(rec,
           root + "longitudinal_axis",
           {rear_end, front_end},
           rerun::Color(148, 163, 184));

  // 3. 前进方向
  double const length = footprint.length();
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
    auto const color = selected                  ? rerun::Color(50, 220, 80)
                       : candidate.feasible      ? rerun::Color(60, 150, 255)
                       : candidate.rejected_pose ? rerun::Color(240, 80, 80)
                                                 : rerun::Color(245, 180, 45);
    std::string const root =
        "gridmap/candidates/candidate_" + std::to_string(i);
    std::vector<rerun::Position3D> points;
    points.reserve(candidate.poses.size());
    for(auto const& pose : candidate.poses)
    {
      points.push_back(position(pose.position, selected ? 0.04F : 0.025F));
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
              rerun::Points3D({position(end.position, 0.04F)})
                  .with_colors({color})
                  .with_radii({0.012F})
                  .with_labels({label})
                  .with_show_labels(false));
      // 原地旋转没有中心位移，用末端朝向箭头显示其结果。
      rec.log(root + "/heading",
              rerun::Arrows3D::from_vectors(
                  {{static_cast<float>(0.12 * std::cos(end.yaw)),
                    static_cast<float>(0.12 * std::sin(end.yaw)),
                    0.0F}})
                  .with_origins({position(end.position, 0.04F)})
                  .with_colors({color})
                  .with_radii({0.004F}));
    }
    if(candidate.rejected_pose)
    {
      rec.log(
          root + "/rejected_pose",
          rerun::Points3D({position(candidate.rejected_pose->position, 0.05F)})
              .with_colors({rerun::Color(255, 40, 40)})
              .with_radii({0.02F})
              .with_labels({"#" + std::to_string(i) + " collision_or_margin"})
              .with_show_labels(false));
    }
  }
}


int
main()
{
  ScopedTimer const total_timer("total (includes Rerun lifetime)");
  try
  {
    std::filesystem::path const grid_map_path =
        "/workspace/algorithms/data/narrow_passage_reverseing2/"
        "frame.json";
    auto map = timed_call(
        "load_grid_map_json",
        [&] { return algorithms::load_grid_map_json(grid_map_path); });
    rerun::RecordingStream rec("narrow_passage");
    timed_call("rerun.spawn", [&] { rec.spawn().exit_on_failure(); });
    timed_call("rerun.blueprint submit",
               [&]
               {
                 rec.log_file_from_path(
                     "/workspace/algorithms/algorithms/narrow_passage.rbl");
               });
    auto const is_occupied = [](std::uint8_t value) { return value != 0; };
    timed_call("grid obstacles build/submit",
               [&] { log_grid_obstacles(rec, "gridmap", map, is_occupied); });

    auto const footprint =
        timed_call("load_footprint",
                   [&] { return algorithms::load_footprint(grid_map_path); });
    std::cout << "siz:" << footprint.vertices.size() << '\n';
    timed_call("footprint build/submit",
               [&] { log_footprint(rec, "gridmap/footprint", footprint); });

    // 示例：车后 0.5 m 至车前 0.5 m，每 0.1 m 查询一个中心线点。
    double const length = footprint.length() + 0.6;
    int count = length / 0.1;
    std::vector<double> longitudinal_samples;
    for(int i = -3; i <= count - 3; ++i)
    {
      longitudinal_samples.push_back(i * 0.1);
    }
    auto lateray_bounday =
        timed_call("build_lateral_boundary",
                   [&]
                   {
                     return algorithms::latter_boundary::build_lateral_boundary(
                         map,
                         footprint.centroid,
                         footprint.forward,
                         longitudinal_samples);
                   });
    timed_call("boundary lines submit",
               [&]
               {
                 log_line(rec,
                          "left_bounds",
                          lateray_bounday.left_points,
                          rerun::Color{255, 0, 0});
                 log_line(rec,
                          "right_bounds",
                          lateray_bounday.right_points,
                          rerun::Color{255, 0, 0});
               });
    // 局部 foot 点
    std::vector<Eigen::Vector2d> footprints;
    footprints.reserve(footprint.vertices.size());
    for(std::size_t i = 0; i < footprint.vertices.size(); ++i)
    {
      Eigen::Vector2d const delta = footprint.vertices[i] - footprint.centroid;
      footprints.emplace_back(delta.dot(footprint.forward),
                              delta.dot(footprint.left()));
    }

    algorithms::Pose start;
    start.position = footprint.centroid;
    start.yaw = std::atan2(footprint.forward.y(), footprint.forward.x());
    auto const result =
        timed_call("planner.plan",
                   [&] { return algorithms::plan(map, start, footprints); });
    timed_call("all candidate paths build/submit",
               [&] { log_candidates(rec, result); });
    if(result.status != algorithms::Status::success || !result.selected)
    {
      std::cerr << "Planning failed: "
                << (result.status == algorithms::Status::start_blocked
                        ? "start_blocked"
                        : "no_path")
                << '\n';
      return 1;
    }
    auto const& selected = result.candidates[*result.selected];
    std::vector<Eigen::Vector2d> path;
    path.reserve(selected.poses.size());
    for(auto const& pose : selected.poses)
    {
      path.push_back(pose.position);
    }
    timed_call("selected path submit",
               [&]
               {
                 log_line(rec,
                          "gridmap/selected_path",
                          path,
                          rerun::Color(50, 220, 80));
               });
    std::cout << "Planning succeeded: reverse_progress="
              << selected.reverse_progress
              << ", min_clearance=" << selected.min_clearance << '\n';
  }
  catch(std::exception const& e)
  {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}