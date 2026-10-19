#include "collision.hh"
#include "geometry.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>


namespace algorithms::local_escape_detail
{
// 计算整个车身到闭合占用格及地图边界的净空，截断到 clearance_cap。
double
footprint_clearance(Context const& ctx, Pose const& pose, SearchTiming& timing)
{
  DetailTimer const detail_timer(timing.details[1]);
  auto polygon = world_footprint(pose, ctx.footprint);
  auto const& g = ctx.grid.geometry();
  Eigen::Vector2d low = polygon.front();
  Eigen::Vector2d high = low;
  for(auto const& p : polygon)
  {
    low = low.cwiseMin(p);
    high = high.cwiseMax(p);
  }
  double distance = std::min({ctx.config.clearance_cap,
                              low.x() - g.origin_x,
                              low.y() - g.origin_y,
                              g.origin_x + g.width * g.resolution - high.x(),
                              g.origin_y + g.height * g.resolution - high.y()});
  if(distance <= 0.0)
  {
    return 0.0;
  }
  auto const index = [](double value, std::size_t size)
  {
    return static_cast<int>(
        std::clamp(std::floor(value), 0.0, static_cast<double>(size - 1)));
  };
  int const xmin =
      index((low.x() - distance - g.origin_x) / g.resolution, g.width);
  int const xmax =
      index((high.x() + distance - g.origin_x) / g.resolution, g.width);
  int const ymin =
      index((low.y() - distance - g.origin_y) / g.resolution, g.height);
  int const ymax =
      index((high.y() + distance - g.origin_y) / g.resolution, g.height);
  for(int y = ymin; y <= ymax; ++y)
  {
    for(int x = xmin; x <= xmax; ++x)
    {
      if(ctx.grid(x, y) < ctx.config.occupied_threshold)
      {
        continue;
      }
      Eigen::Vector2d cell_low(g.origin_x + x * g.resolution,
                               g.origin_y + y * g.resolution);
      Eigen::Vector2d cell_high =
          cell_low + Eigen::Vector2d::Constant(g.resolution);
      if(((low - cell_high).cwiseMax(cell_low - high).cwiseMax(0.0)).norm() >
         distance)
      {
        continue;
      }
      ++timing.cell_distance_calls;
      distance =
          std::min(distance, cell_distance(polygon, cell_low, cell_high));
      if(distance <= epsilon)
      {
        return 0.0;
      }
    }
  }
  return distance;
}

// 每个区间以中点净空减去半段运动上界证明连续安全；失败时只保留已验证的端点。
Candidate
extend(Context const& ctx,
       Candidate candidate,
       Motion const& motion,
       SearchTiming& timing)
{
  DetailTimer const detail_timer(timing.details[2]);
  candidate.motions.push_back(motion);
  Pose const from = candidate.poses.back();
  double const angle = motion.rotation + motion.curvature * motion.distance;
  double const bound = std::abs(motion.distance) + ctx.radius * std::abs(angle);
  double const step_count = std::max(1.0, std::ceil(bound / ctx.step_bound));
  if(!std::isfinite(step_count) || step_count > 100000)
  {
    throw std::invalid_argument(
        "Local escape: motion requires too many collision samples");
  }

  auto const steps = static_cast<std::size_t>(step_count);
  // 任意车身点从区间中点运动到区间内任意时刻的位移上界。
  double const half_motion_bound = bound / steps * 0.5;

  for(std::size_t i = 0; i < steps; ++i)
  {
    Pose const midpoint = advance(from, motion, (i + 0.5) / steps);
    double const clearance = footprint_clearance(ctx, midpoint, timing);

    double const lower_bound = std::max(0.0, clearance - half_motion_bound);
    if(lower_bound <= ctx.config.safety_margin + epsilon)
    {
      candidate.rejected_pose = midpoint;
      candidate.reason = "collision_or_margin";
      return candidate;
    }
    candidate.min_clearance = std::min(candidate.min_clearance, lower_bound);
    candidate.poses.push_back(
        advance(from, motion, static_cast<double>(i + 1) / steps));
  }

  return candidate;
}
} // namespace algorithms::local_escape_detail
