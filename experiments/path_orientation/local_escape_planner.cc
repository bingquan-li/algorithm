#include "local_escape_planner.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <stdexcept>

namespace local_escape
{
namespace
{
using Vec = Eigen::Vector2d;
constexpr double epsilon = 1e-10;

double
cross(Vec const& a, Vec const& b)
{
  return a.x() * b.y() - a.y() * b.x();
}

double
point_segment_distance(Vec const& p, Vec const& a, Vec const& b)
{
  Vec const edge = b - a;
  double const t =
      edge.squaredNorm() == 0.0
          ? 0.0
          : std::clamp((p - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
  return (p - a - t * edge).norm();
}

bool
inside(Vec const& p, std::span<Vec const> polygon)
{
  bool result = false;
  for(std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    auto const& a = polygon[i];
    auto const& b = polygon[j];
    if((a.y() > p.y()) != (b.y() > p.y()) &&
       p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
      result = !result;
  }
  return result;
}

bool
segments_intersect(Vec const& a, Vec const& b, Vec const& c, Vec const& d)
{
  double const ab_c = cross(b - a, c - a), ab_d = cross(b - a, d - a);
  double const cd_a = cross(d - c, a - c), cd_b = cross(d - c, b - c);
  if(((ab_c > 0 && ab_d < 0) || (ab_c < 0 && ab_d > 0)) &&
     ((cd_a > 0 && cd_b < 0) || (cd_a < 0 && cd_b > 0)))
    return true;
  return point_segment_distance(a, c, d) <= epsilon ||
         point_segment_distance(b, c, d) <= epsilon ||
         point_segment_distance(c, a, b) <= epsilon ||
         point_segment_distance(d, a, b) <= epsilon;
}

// 对任意简单（含凹）多边形计算与闭合障碍格的距离。
double
cell_distance(std::span<Vec const> polygon, Vec const& lo, Vec const& hi)
{
  std::array<Vec, 4> const corners{lo,
                                   Vec(hi.x(), lo.y()),
                                   hi,
                                   Vec(lo.x(), hi.y())};
  if(inside(lo, polygon))
    return 0.0; // 整个障碍格位于 footprint 内部
  double result = std::numeric_limits<double>::infinity();
  for(std::size_t i = 0; i < polygon.size(); ++i)
  {
    auto const& a = polygon[i];
    auto const& b = polygon[(i + 1) % polygon.size()];
    Vec const offset = (lo - a).cwiseMax(a - hi).cwiseMax(0.0);
    result = std::min(result, offset.norm());
    if(result == 0.0)
      return 0.0;
    // Slab 交点检查，包含边缘/角点接触。
    double enter = 0.0, leave = 1.0;
    for(int axis = 0; axis < 2; ++axis)
    {
      double const delta = b[axis] - a[axis];
      if(delta == 0.0)
      {
        if(a[axis] < lo[axis] || a[axis] > hi[axis])
          enter = 2.0;
      }
      else
      {
        double t0 = (lo[axis] - a[axis]) / delta;
        double t1 = (hi[axis] - a[axis]) / delta;
        if(t0 > t1)
          std::swap(t0, t1);
        enter = std::max(enter, t0);
        leave = std::min(leave, t1);
      }
    }
    if(enter <= leave)
      return 0.0;
    for(auto const& corner : corners)
      result = std::min(result, point_segment_distance(corner, a, b));
  }
  return result;
}

Vec
direction(Pose const& pose)
{
  return {std::cos(pose.yaw), std::sin(pose.yaw)};
}

Pose
advance(Pose const& start, Motion const& motion, double fraction)
{
  Pose result = start;
  double const ds = motion.distance * fraction;
  double const angle = motion.rotation * fraction + motion.curvature * ds;
  // 稳定的圆弧积分；distance=0 时自然退化为原地旋转。
  double const half = 0.5 * angle;
  double const sinc =
      std::abs(half) < 1e-8 ? 1.0 - half * half / 6.0 : std::sin(half) / half;
  result.position +=
      ds * sinc * Vec(std::cos(start.yaw + half), std::sin(start.yaw + half));
  result.yaw += angle;
  return result;
}

std::pair<double, double>
lateral_extent(std::span<Vec const> footprint, double x)
{
  double low = std::numeric_limits<double>::infinity();
  double high = -low;
  // 遍历机器人的每一条边
  for(std::size_t i = 0; i < footprint.size(); ++i)
  {
    // 每次取两个相邻顶点
    auto const& a = footprint[i];
    auto const& b = footprint[(i + 1) % footprint.size()];
    // 排除不可能相交的点
    if(x < std::min(a.x(), b.x()) || x > std::max(a.x(), b.x()))
    {
      continue;
    }
    // 如果是平行的 ==> 其实是重合了。。。
    if(a.x() == b.x())
    {
      low = std::min({low, a.y(), b.y()});
      high = std::max({high, a.y(), b.y()});
    }
    else
    {
      // 线性插值求交点，自己以为是求解一次函数方程，然后计算交点。。。其实用线性插值理解会更好
      double const y = a.y() + (x - a.x()) * (b.y() - a.y()) / (b.x() - a.x());
      low = std::min(low, y);
      high = std::max(high, y);
    }
  }
  return {low, high};
}

// 构建左右净空分布，并计算边界变化趋势和建议曲率
BoundaryProfile
profile(algorithms::GridMap<std::uint8_t> const& grid,
        Pose const& pose,
        std::span<Vec const> footprint,
        Config const& config,
        double travel)
{
  // 这里的 footprint 按前文约定，使用车身坐标系。并非是在 gridmap 中的坐标！
  // 纵向查询范围，机器人自身坐标系
  double xmin = footprint.front().x();
  double xmax = xmin;
  for(auto const& p : footprint)
  {
    xmin = std::min(xmin, p.x());
    xmax = std::max(xmax, p.x());
  }
  BoundaryProfile result;
  // 当前车身及运动后车身所在区域都查询，不能只查询当前车长。
  // 这个范围只按直线平移构造，不等于旋转运动的完整扫掠包络。
  double const begin = xmin + std::min(0.0, travel);
  double const end = xmax + std::max(0.0, travel);

  constexpr std::size_t count = 41;

  for(std::size_t i = 0; i < count; ++i)
  {
    result.longitudinal_samples.push_back(begin +
                                          (end - begin) * i / (count - 1));
  }

  result.widths = build_lateral_boundary(grid,
                                         pose.position,
                                         direction(pose),
                                         result.longitudinal_samples,
                                         config.occupied_threshold);

  //  分子。
  double numerator = 0.0;
  // 分母
  double denominator = 0.0;
  // cov：covariance 的缩写，表示协方差。两个变量相对于各自均值的变化，是否同向。
  double cov_left = 0.0;
  double cov_right = 0.0;
  // variance：表示方差。自身有多分散
  double variance = 0.0;
  // 由于采样均匀且包含两端，所以这个值就是所有 \(s_i\) 的平均值。
  double const mean_s = (begin + end) * 0.5;
  for(std::size_t i = 0; i < count; ++i)
  {
    double const s = result.longitudinal_samples[i];

    double const body_x = std::clamp(s - travel, xmin, xmax);

    // lateral_extent: 求车身在指定纵向位置 x 处的横向范围
    // 用直线 \(X=x\) 切车身多边形，该截面的最小、最大横坐标 \(y\) 是多少？, 是沿横轴切，使用的是车身坐标系！
    // low:最右侧的 y,high：最左侧的 y
    // 垂直于车身 x 轴、平行于车身 y 轴的
    auto const [low, high] = lateral_extent(footprint, body_x);

    // 从障碍距离中扣除车身，得到左右净空
    // 左右障碍物的距离 - 车身的宽度 - 安全余量
    double const left =
        result.widths.left_width[i] - high - config.safety_margin;
    double const right =
        result.widths.right_width[i] + low - config.safety_margin;

    result.left_clearance.push_back(left);
    result.right_clearance.push_back(right);

    // 左右斜率：净空沿车身纵向如何变化
    // 这里做的是一个最小二乘法拟合
    double const centered_s = s - mean_s;
    cov_left += centered_s * left;
    cov_right += centered_s * right;
    variance += centered_s * centered_s;
    // 也就是只选择预计运动完成后，车身占据的纵向范围
    // 这也说明该曲率拟合主要针对运动结束时的车身位置，不是逐时刻的轨迹优化
    // 拟合运动结束时的车身侧移
    // 只用运动后车身所在区间拟合曲率
    // travel： 旅行，行程，这个 travel 表示的是后退的距离，不是自己一开始理解的向后延长的边界
    if(s >= xmin + travel && s <= xmax + travel)
    {
      // 小角度下车身截面的侧移约 k*(travel^2/2 + travel*body_x)
      // 包含车头外摆与车尾侧移，且使用前后各截面的边界变化
      // desired： 这个截面希望向哪边移动多少，尽可能的向净空多的一侧移动
      double const desired = std::clamp(0.5 * (left - right), -0.25, 0.25);
      double const coefficient = 0.5 * travel * travel + travel * body_x;
      double const weight = 1.0 / (0.05 + std::max(0.0, std::min(left, right)));
      numerator += weight * coefficient * desired;
      denominator += weight * coefficient * coefficient;
    }
  }
  result.left_slope = cov_left / variance;
  result.right_slope = cov_right / variance;
  result.preferred_curvature =
      std::clamp(denominator > epsilon ? numerator / denominator : 0.0,
                 -config.max_curvature,
                 config.max_curvature);
  return result;
}



std::vector<double>
curvatures(BoundaryProfile const& boundary, Config const& config)
{
  double const preferred = boundary.preferred_curvature;
  std::vector<double> values{0.0, preferred, preferred * 0.5};
  for(double const factor : {-1.0, -0.5, -0.2, 0.2, 0.5, 1.0})
    values.push_back(factor * config.max_curvature);
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(),
                           values.end(),
                           [](double a, double b)
                           { return std::abs(a - b) < epsilon; }),
               values.end());
  std::stable_sort(
      values.begin(),
      values.end(),
      [preferred](double a, double b)
      { return std::abs(a - preferred) < std::abs(b - preferred); });
  return values;
}

void
validate(algorithms::GridMap<std::uint8_t> const& grid,
         Pose const& start,
         std::span<Vec const> footprint,
         Config const& config)
{
  auto const finite_positive = [](double value)
  { return std::isfinite(value) && value > 0.0; };
  auto const& g = grid.geometry();
  if(!start.position.allFinite() || !std::isfinite(start.yaw) ||
     footprint.size() < 3 || g.width == 0 || g.height == 0 ||
     g.width > std::numeric_limits<int>::max() ||
     g.height > std::numeric_limits<int>::max() ||
     !std::isfinite(g.origin_x + g.width * g.resolution) ||
     !std::isfinite(g.origin_y + g.height * g.resolution) ||
     !finite_positive(config.reverse_length) ||
     !finite_positive(config.min_reverse_progress) ||
     config.min_reverse_progress > config.reverse_length ||
     !std::isfinite(config.max_curvature) || config.max_curvature < 0 ||
     !std::isfinite(config.safety_margin) || config.safety_margin < 0 ||
     !finite_positive(config.max_corner_step) ||
     !finite_positive(config.clearance_cap) ||
     config.clearance_cap <= config.safety_margin ||
     config.occupied_threshold == 0 || config.forward_beam_width == 0)
  {
    throw std::invalid_argument(
        "Local escape: invalid geometry, pose or configuration");
  }
  for(double value : config.forward_lengths)
  {
    if(!finite_positive(value))
    {
      throw std::invalid_argument("Local escape: invalid forward length");
    }
  }
  for(double value : config.rotation_angles)
  {
    if(!std::isfinite(value) || value == 0.0 ||
       std::abs(value) > std::numbers::pi)
    {
      throw std::invalid_argument("Local escape: invalid rotation angle");
    }
  }
  double area = 0.0;
  for(std::size_t i = 0; i < footprint.size(); ++i)
  {
    Vec const& a = footprint[i];
    Vec const& b = footprint[(i + 1) % footprint.size()];
    if(!a.allFinite() || !std::isfinite(a.norm()) || (a - b).norm() <= epsilon)
    {
      throw std::invalid_argument("Local escape: invalid footprint edge");
    }
    area += cross(a, b);
    for(std::size_t j = i + 2; j < footprint.size(); ++j)
    {
      if(i == 0 && j == footprint.size() - 1)
      {
        continue;
      }
      if(segments_intersect(a,
                            b,
                            footprint[j],
                            footprint[(j + 1) % footprint.size()]))
      {
        throw std::invalid_argument(
            "Local escape: footprint must be a simple polygon");
      }
    }
  }
  if(!std::isfinite(area) || std::abs(area) <= epsilon)
  {
    throw std::invalid_argument("Local escape: degenerate footprint");
  }
}
} // namespace

std::vector<Vec>
world_footprint(Pose const& pose, std::span<Vec const> footprint)
{
  Vec const forward = direction(pose);
  Vec const left(-forward.y(), forward.x());
  std::vector<Vec> result;
  result.reserve(footprint.size());
  for(auto const& p : footprint)
  {
    result.emplace_back(pose.position + p.x() * forward + p.y() * left);
  }
  return result;
}


double
footprint_clearance(algorithms::GridMap<std::uint8_t> const& grid,
                    Pose const& pose,
                    std::span<Vec const> footprint,
                    double cap,
                    std::uint8_t occupied_threshold)
{
  auto const polygon = world_footprint(pose, footprint);
  auto const& g = grid.geometry();
  Vec low = polygon.front();
  Vec high = low;
  for(auto const& p : polygon)
  {
    low = low.cwiseMin(p);
    high = high.cwiseMax(p);
  }
  double distance = std::min({cap,
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
      if(grid(x, y) < occupied_threshold)
      {
        continue;
      }
      Vec const cell_low(g.origin_x + x * g.resolution,
                         g.origin_y + y * g.resolution);
      Vec const cell_high = cell_low + Vec::Constant(g.resolution);
      if(((low - cell_high).cwiseMax(cell_low - high).cwiseMax(0.0)).norm() >
         distance)
      {
        continue;
      }
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


Result
plan(algorithms::GridMap<std::uint8_t> const& grid,
     Pose const& start,
     std::span<Vec const> footprint,
     Config const& config)
{
  // 接受闭合或未闭合的轮廓，内部统一使用无重复端点形式。
  if(footprint.size() > 1 && footprint.front() == footprint.back())
  {
    footprint = footprint.first(footprint.size() - 1);
  }

  validate(grid, start, footprint, config);

  Result result;
  result.boundary =
      profile(grid, start, footprint, config, -config.reverse_length);

  double const initial_clearance =
      footprint_clearance(grid,
                          start,
                          footprint,
                          config.clearance_cap,
                          config.occupied_threshold);
  if(initial_clearance <= config.safety_margin + epsilon)
  {
    result.status = Status::start_blocked;
    return result;
  }

  double radius = 0.0;
  for(auto const& p : footprint)
  {
    radius = std::max(radius, p.norm());
  }
  double const step_bound =
      std::min({config.max_corner_step,
                grid.geometry().resolution * 0.5,
                (config.clearance_cap - config.safety_margin) * 0.5});
  Candidate root;
  root.poses.push_back(start);
  root.min_clearance = initial_clearance;
  auto const extend = [&](Candidate candidate, Motion const& motion)
  {
    candidate.motions.push_back(motion);
    Pose const from = candidate.poses.back();
    double const angle = motion.rotation + motion.distance * motion.curvature;
    // 任何车身点的路程 <= |ds| + radius*|dtheta|。
    double const bound = std::abs(motion.distance) + radius * std::abs(angle);
    double const step_count = std::max(1.0, std::ceil(bound / step_bound));
    if(!std::isfinite(step_count) || step_count > 100000)
    {
      throw std::invalid_argument(
          "Local escape: motion requires too many collision samples");
    }
    auto const steps = static_cast<std::size_t>(step_count);
    double const half_motion_bound = bound / steps * 0.5;
    for(std::size_t i = 0; i < steps; ++i)
    {
      Pose const midpoint = advance(from, motion, (i + 0.5) / steps);
      double const clearance = footprint_clearance(grid,
                                                   midpoint,
                                                   footprint,
                                                   config.clearance_cap,
                                                   config.occupied_threshold);
      // 距离函数的 Lipschitz 界：中点距离减最大位移，覆盖整个采样区间，
      // 包括平移跨薄障碍、原地旋转角点扫过障碍的情况。
      double const lower_bound = std::max(0.0, clearance - half_motion_bound);
      if(lower_bound <= config.safety_margin + epsilon)
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
  };

  auto const record = [&](Candidate candidate, bool terminal)
  {
    Pose const& end = candidate.poses.back();
    Vec const offset = end.position - start.position;
    Vec const forward = direction(start);
    Vec const left(-forward.y(), forward.x());
    candidate.reverse_progress = -offset.dot(forward);
    candidate.feasible =
        terminal && !candidate.rejected_pose &&
        candidate.reverse_progress >= config.min_reverse_progress;
    if(!candidate.rejected_pose)
      candidate.reason = !terminal            ? "reposition_prefix"
                         : candidate.feasible ? "feasible"
                                              : "insufficient_reverse_progress";
    double forward_distance = 0.0;
    double turning = 0.0;
    double curvature_effort = 0.0;
    for(auto const& motion : candidate.motions)
    {
      forward_distance += std::max(0.0, motion.distance);
      turning += std::abs(motion.rotation);
      curvature_effort += std::abs(motion.distance * motion.curvature);
    }
    candidate.score =
        4.0 * candidate.reverse_progress + 2.0 * candidate.min_clearance -
        forward_distance - 0.25 * turning - 0.15 * curvature_effort -
        0.4 * std::abs(offset.dot(left)) - 0.2 * std::abs(end.yaw - start.yaw);
    std::size_t const index = result.candidates.size();
    if(candidate.feasible &&
       (!result.selected ||
        candidate.score > result.candidates[*result.selected].score))
    {
      result.selected = index;
    }
    result.candidates.push_back(std::move(candidate));
  };
  auto const try_reverse = [&](Candidate const& prefix)
  {
    auto const boundaries = profile(grid,
                                    prefix.poses.back(),
                                    footprint,
                                    config,
                                    -config.reverse_length);
    for(double curvature : curvatures(boundaries, config))
    {
      record(extend(prefix, {-config.reverse_length, curvature, 0.0}), true);
    }
  };
  auto const try_rotations = [&](Candidate const& prefix)
  {
    if(!config.allow_in_place_rotation)
    {
      return;
    }
    for(double angle : config.rotation_angles)
    {
      auto rotated = extend(prefix, {0.0, 0.0, angle});
      record(rotated, false);
      if(!rotated.rejected_pose)
      {
        try_reverse(rotated);
      }
    }
  };

  try_reverse(root);
  // 已有完整后退轨迹时优先执行，不引入不必要的前进/旋转。
  if(!result.selected)
  {
    try_rotations(root);
    std::vector<Candidate> forward_prefixes;
    for(double length : config.forward_lengths)
    {
      auto const boundaries = profile(grid, start, footprint, config, length);
      for(double curvature : curvatures(boundaries, config))
      {
        auto forward = extend(root, {length, curvature, 0.0});
        record(forward, false);
        if(!forward.rejected_pose)
        {
          try_reverse(forward);
          forward_prefixes.push_back(std::move(forward));
        }
      }
    }
    // 有限宽度搜索前进 + 原地旋转 + 后退；每次重新计算当地左右边界。
    std::stable_sort(forward_prefixes.begin(),
                     forward_prefixes.end(),
                     [](Candidate const& a, Candidate const& b)
                     { return a.min_clearance > b.min_clearance; });
    for(std::size_t i = 0;
        i < std::min(config.forward_beam_width, forward_prefixes.size());
        ++i)
    {
      try_rotations(forward_prefixes[i]);
    }
  }
  result.status = result.selected ? Status::success : Status::no_path;
  return result;
}
} // namespace local_escape
