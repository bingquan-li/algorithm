#include "local_escape_planner.hh"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using local_escape::Pose;
using Vec = Eigen::Vector2d;
using Grid = algorithms::GridMap<std::uint8_t>;

void
check(bool condition, char const* message)
{
  if(!condition)
  {
    throw std::runtime_error(message);
  }
}
void
near(double value, double expected)
{
  check(std::isfinite(value) && std::abs(value - expected) < 1e-8,
        "Unexpected geometric distance");
}
Grid
empty_map()
{
  return Grid({200, 200, 0.02, -2.0, -2.0});
}
std::vector<Vec>
rectangle(double half_length = 0.2, double half_width = 0.1)
{
  return {{-half_length, -half_width},
          {half_length, -half_width},
          {half_length, half_width},
          {-half_length, half_width}};
}
void
obstacle(Grid& grid, double xmin, double xmax, double ymin, double ymax)
{
  auto const& g = grid.geometry();
  for(std::size_t y = 0; y < g.height; ++y)
  {
    for(std::size_t x = 0; x < g.width; ++x)
    {
      double const cx = g.origin_x + (x + 0.5) * g.resolution;
      double const cy = g.origin_y + (y + 0.5) * g.resolution;
      if(cx >= xmin && cx < xmax && cy >= ymin && cy < ymax)
      {
        grid(x, y) = 2;
      }
    }
  }
}
void
verify_selected(Grid const& grid,
                std::span<Vec const> footprint,
                local_escape::Result const& result,
                local_escape::Config const& config)
{
  check(result.status == local_escape::Status::success &&
            result.selected.has_value(),
        "Expected a selected path");
  auto const& candidate = result.candidates[*result.selected];
  check(candidate.feasible && !candidate.rejected_pose,
        "Selected path is rejected");
  check(candidate.reverse_progress >= config.min_reverse_progress,
        "No net reverse progress");
  check(candidate.min_clearance > config.safety_margin,
        "Insufficient continuous clearance");
  // 独立加密检查已返回位姿之间的中间轮廓；覆盖旋转和前后换向。
  for(std::size_t i = 0; i < candidate.poses.size(); ++i)
  {
    auto const& a = candidate.poses[(i != 0U) ? i - 1 : i];
    auto const& b = candidate.poses[i];
    for(int j = 0; j <= 4; ++j)
    {
      double const t = j / 4.0;
      Pose const p{(1 - t) * a.position + t * b.position,
                   (1 - t) * a.yaw + t * b.yaw};
      check(local_escape::footprint_clearance(grid,
                                              p,
                                              footprint,
                                              config.clearance_cap) >
                config.safety_margin,
            "Dense independent footprint validation failed");
    }
  }
}

template <typename Function>
void
reject(Function function)
{
  try
  {
    function();
  }
  catch(std::invalid_argument const&)
  {
    return;
  }
  throw std::runtime_error("Expected invalid_argument");
}
} // namespace

int
main()
{
  std::cout << "Running local escape planner tests; please wait...\n"
               "This test executable does not launch Rerun.\n"
               "For visualization, run: "
               "/workspace/algorithms/build/experiments/path_orientation/"
               "later_boundary\n"
            << std::flush;
  try
  {
    auto grid = empty_map();
    auto const body = rectangle();
    local_escape::Config config;
    // 开阔空间应选直线，输出完整后退距离，且不做无意义的前进/旋转。
    auto result = local_escape::plan(grid, {}, body, config);
    verify_selected(grid, body, result, config);
    auto const& straight = result.candidates[*result.selected];
    check(straight.motions.size() == 1, "Open space should use one primitive");
    near(straight.motions.front().curvature, 0);
    near(straight.poses.back().position.x(), -config.reverse_length);
    near(straight.poses.back().position.y(), 0);

    // 完整格子位于机器人内部，不能只查 footprint 边缘。
    obstacle(grid, -0.02, 0.02, -0.02, 0.02);
    near(local_escape::footprint_clearance(grid, {}, body, 1.0), 0);
    result = local_escape::plan(grid, {}, body, config);
    check(result.status == local_escape::Status::start_blocked &&
              !result.selected,
          "Interior obstacle missed");
    grid = empty_map();
    near(local_escape::footprint_clearance(grid, {{1.9, 0}, 0}, body, 1.0), 0);
    obstacle(grid, 0.4, 0.42, -0.02, 0.02);
    near(local_escape::footprint_clearance(grid, {}, body, 1.0), 0.2);
    // 凹多边形的缺口不应被凸包误封闭。
    std::vector<Vec> const concave{{-.4, -.4},
                                   {.4, -.4},
                                   {.4, -.2},
                                   {-.2, -.2},
                                   {-.2, .4},
                                   {-.4, .4}};
    grid = empty_map();
    obstacle(grid, 0.0, 0.02, 0.0, 0.02);
    near(local_escape::footprint_clearance(grid, {}, concave, 1.0), 0.2);

    // 后方一侧障碍：直线失败，曲线后退成功，镜像场景应反向转弯。
    for(double sign : {1.0, -1.0})
    {
      grid = empty_map();
      obstacle(grid,
               -0.9,
               -0.5,
               sign > 0 ? -0.3 : 0.04,
               sign > 0 ? -0.04 : 0.3);
      result = local_escape::plan(grid, {}, body, config);
      verify_selected(grid, body, result, config);
      auto const& curved = result.candidates[*result.selected];
      check(curved.motions.size() == 1 &&
                curved.motions.front().curvature * sign > 0,
            "Expected a boundary-guided reverse curve");
      check(result.boundary.preferred_curvature * sign > 0,
            "Boundary preference has wrong sign");
      bool straight_rejected = false;
      for(auto const& c : result.candidates)
      {
        if(c.motions.size() == 1 && c.motions[0].curvature == 0.0 &&
           c.rejected_pose)
        {
          straight_rejected = true;
        }
      }
      check(straight_rejected,
            "Straight reverse should collide in curved scenario");
    }

    // 障碍横跨后方：先前进改变姿态，再旋转/曲线后退才可能绕开。
    grid = empty_map();
    obstacle(grid, -0.46, -0.36, -0.18, 0.18);
    result = local_escape::plan(grid, {}, body, config);
    verify_selected(grid, body, result, config);
    auto const& recovery = result.candidates[*result.selected];
    check(recovery.motions.size() > 1,
          "Recovery should require multiple motions");
    bool moved_forward = false;
    for(auto const& m : recovery.motions)
    {
      moved_forward |= m.distance > 0;
    }
    check(moved_forward, "Recovery should first move forward");

    // 允许原地旋转后再直线后退；禁用旋转后，同一候选集合应报告不可行。
    grid = empty_map();
    obstacle(grid, -0.9, -0.36, -0.3, -0.04);
    auto rotation_config = config;
    rotation_config.max_curvature = 0;
    rotation_config.forward_lengths.clear();
    rotation_config.rotation_angles = {-0.6, 0.6};
    result = local_escape::plan(grid, {}, body, rotation_config);
    verify_selected(grid, body, result, rotation_config);
    check(result.candidates[*result.selected].motions.front().rotation != 0,
          "Expected in-place rotation");
    rotation_config.allow_in_place_rotation = false;
    result = local_escape::plan(grid, {}, body, rotation_config);
    check(!result.selected && result.status == local_escape::Status::no_path,
          "Rotation disable ignored");

    // 原地旋转起终点都安全，但长车身的前角在中途扫过一个小障碍。
    grid = empty_map();
    auto const long_body = rectangle(0.5, 0.05);
    obstacle(grid, -0.7, -0.6, -1, 1); // 先阻止直接后退，触发旋转搜索
    obstacle(grid, 0.46, 0.48, 0.16, 0.18);
    auto swept_config = config;
    swept_config.max_curvature = 0;
    swept_config.forward_lengths.clear();
    swept_config.rotation_angles = {0.7};
    check(local_escape::footprint_clearance(grid, {}, long_body, 0.3) >
                  config.safety_margin &&
              local_escape::footprint_clearance(grid,
                                                {{0, 0}, 0.7},
                                                long_body,
                                                0.3) > config.safety_margin,
          "Rotation fixture endpoints must be clear");
    result = local_escape::plan(grid, {}, long_body, swept_config);
    bool swept_collision = false;
    for(auto const& c : result.candidates)
    {
      if(c.motions.size() == 1 && c.motions.front().rotation == 0.7 &&
         c.rejected_pose)
      {
        swept_collision = true;
      }
    }
    check(swept_collision && !result.selected,
          "Rotation swept collision was missed");

    // 完全封闭：不允许把前进前缀或碰撞前的后退前缀当作成功。
    grid = empty_map();
    obstacle(grid, -0.4, -0.3, -1, 1);
    obstacle(grid, 0.3, 0.4, -1, 1);
    obstacle(grid, -1, 1, -0.3, -0.2);
    obstacle(grid, -1, 1, 0.2, 0.3);
    result = local_escape::plan(grid, {}, body, config);
    check(result.status == local_escape::Status::no_path && !result.selected,
          "Trapped robot must stop");

    // 一格厚障碍不能因用户配置较大采样步长而被跳过。
    grid = empty_map();
    obstacle(grid, -0.46, -0.44, -1, 1);
    auto thin_config = config;
    thin_config.max_corner_step = 10;
    thin_config.max_curvature = 0;
    thin_config.forward_lengths.clear();
    thin_config.allow_in_place_rotation = false;
    result = local_escape::plan(grid, {}, body, thin_config);
    check(!result.selected &&
              result.candidates.front().rejected_pose.has_value(),
          "Thin obstacle was skipped");

    grid = empty_map();
    reject(
        [&]
        {
          (void)local_escape::plan(
              grid,
              {{0, 0}, std::numeric_limits<double>::quiet_NaN()},
              body);
        });
    auto bad = config;
    bad.max_corner_step = 0;
    reject([&] { (void)local_escape::plan(grid, {}, body, bad); });
    std::vector<Vec> const crossed{{-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
    reject([&] { (void)local_escape::plan(grid, {}, crossed); });
    auto closed = body;
    closed.push_back(body.front());
    result = local_escape::plan(grid, {}, closed);
    verify_selected(grid, body, result, config);
    std::cout << "Local escape planner tests passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
