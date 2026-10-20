#include "local_escape_planner.hh"
#include "later_boundary_visualization.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
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
    {
      result = !result;
    }
  }
  return result;
}

bool
segments_intersect(Vec const& a, Vec const& b, Vec const& c, Vec const& d)
{
  double const ab_c = cross(b - a, c - a);
  double const ab_d = cross(b - a, d - a);
  double const cd_a = cross(d - c, a - c);
  double const cd_b = cross(d - c, b - c);
  if(((ab_c > 0 && ab_d < 0) || (ab_c < 0 && ab_d > 0)) &&
     ((cd_a > 0 && cd_b < 0) || (cd_a < 0 && cd_b > 0)))
  {
    return true;
  }
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
  {
    return 0.0; // 整个障碍格位于 footprint 内部
  }
  double result = std::numeric_limits<double>::infinity();
  for(std::size_t i = 0; i < polygon.size(); ++i)
  {
    auto const& a = polygon[i];
    auto const& b = polygon[(i + 1) % polygon.size()];
    Vec const offset = (lo - a).cwiseMax(a - hi).cwiseMax(0.0);
    result = std::min(result, offset.norm());
    if(result == 0.0)
    {
      return 0.0;
    }
    // Slab 交点检查，包含边缘/角点接触。
    double enter = 0.0;
    double leave = 1.0;
    for(int axis = 0; axis < 2; ++axis)
    {
      double const delta = b[axis] - a[axis];
      if(delta == 0.0)
      {
        if(a[axis] < lo[axis] || a[axis] > hi[axis])
        {
          enter = 2.0;
        }
      }
      else
      {
        double t0 = (lo[axis] - a[axis]) / delta;
        double t1 = (hi[axis] - a[axis]) / delta;
        if(t0 > t1)
        {
          std::swap(t0, t1);
        }
        enter = std::max(enter, t0);
        leave = std::min(leave, t1);
      }
    }
    if(enter <= leave)
    {
      return 0.0;
    }
    for(auto const& corner : corners)
    {
      result = std::min(result, point_segment_distance(corner, a, b));
    }
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
// 在机器人导航、路径规划和几何处理中，profile 通常表示某个物理量沿特定维度的分布或变化情况。
/*
场景	中文含义	示例
日常英语	侧面、轮廓	face profile（面部侧面轮廓）
用户系统	个人资料、配置档案	UserProfile
数据分析	特征描述、分布情况	DataProfile
机器人规划	沿某个维度的变化分布	VelocityProfile
软件性能分析	性能剖析	CPU profiling
*/
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



// curvatures() 的作用是：生成一组值得尝试的曲率，并按照接近边界建议曲率
// 的程度排序，供规划器逐个尝试。
// 它不判断碰撞，也不直接选出最终曲率；它负责提供“候选项”。
/*
 1. 为什么需要一组曲率

  profile() 根据左右净空计算了一个建议：

  boundary.preferred_curvature

  但这个建议来自近似拟合，没有完整考虑实际圆弧运动的扫掠区域，因此可能出
  现：

  - 建议曲率对应的轨迹发生碰撞。
  - 更小的曲率反而可以安全通过。
  - 直线就能完成后退。
  - 必须尝试另一个方向才能找到可行解。

  所以规划器不会只尝试 preferred_curvature，而是用它引导一组离散候选。
*/
std::vector<double>
curvatures(BoundaryProfile const& boundary, Config const& config)
{
  double const preferred = boundary.preferred_curvature;
  // 第一组： 直线、建议曲率和较温和的建议
  std::vector<double> values{0.0, preferred, preferred * 0.5};

  // 第二组：固定比例的正负曲率
  // 它补充了三个弯曲程度、两个方向的候选：
  for(double const factor : {-1.0, -0.5, -0.2, 0.2, 0.5, 1.0})
  {
    values.push_back(factor * config.max_curvature);
  }
  // 排序和去掉重复候选
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(),
                           values.end(),
                           [](double a, double b)
                           { return std::abs(a - b) < epsilon; }),
               values.end());

  //  第二次排序：优先尝试更接近建议的曲率
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


// > 根据左右障碍分布生成有针对性的运动候选，用整车轮廓检查整个运动过程，
// > 再从满足脱困条件的候选中选出评分最高的一条。
Result
plan(algorithms::GridMap<std::uint8_t> const& grid,
     Pose const& start,
     std::span<Vec const> footprint,
     Config const& config)
{
  // 接受闭合或未闭合的轮廓，内部统一使用无重复端点形式。
  // 首先处理闭合轮廓， 后面的几何算法通过取模连接最后一个点和第一个点，所以不需要重复端点。
  if(footprint.size() > 1 && footprint.front() == footprint.back())
  {
    footprint = footprint.first(footprint.size() - 1);
  }

  //
  validate(grid, start, footprint, config);
  lateral_boundary_viz::begin(grid, start, footprint);

  Result result;
  // 这是初始位姿下，面向后退动作的左右边界信息。它主要用于解释环境和引导候选，不能单独证明轨迹安全。
  result.boundary =
      profile(grid, start, footprint, config, -config.reverse_length);
  lateral_boundary_viz::boundary(start.position, direction(start),
                                  result.boundary.longitudinal_samples, result.boundary.widths);

  // 起点安全检查
  double const initial_clearance =
      footprint_clearance(grid,
                          start,
                          footprint,
                          config.clearance_cap,
                          config.occupied_threshold);
  // 这里要求严格大于安全余量。也就是说，即使起点尚未发生实际碰撞，只要安全
  // 余量不足，也返回 start_blocked。
  if(initial_clearance <= config.safety_margin + epsilon)
  {
    result.status = Status::start_blocked;
    lateral_boundary_viz::finish(result, footprint);
    return result;
  }


  // 首先计算车身最大半径：它是车身上最远点到旋转中心的距离。
  // 这里的 footprint 已经不是世界坐标了，而是相对坐标了，应该。。。。
  double radius = 0.0;
  for(auto const& p : footprint)
  {
    radius = std::max(radius, p.norm());
  }
  /*
      每个采样区间允许的运动上界是：
      step_bound = min(
          max_corner_step,
          resolution * 0.5,
          (clearance_cap - safety_margin) * 0.5
      );
      三项分别控制：
      - 用户配置的车身点运动尺度。
      - 相对于地图栅格大小的采样尺度。
      - 距离截断值与安全余量之间可用的判断空间。
  */
  double const step_bound =
      std::min({config.max_corner_step,
                grid.geometry().resolution * 0.5,
                (config.clearance_cap - config.safety_margin) * 0.5});
  Candidate root;
  root.poses.push_back(start);
  root.min_clearance = initial_clearance;

  // 把一个运动接到候选轨迹后面
  // candidate 按值传入，所以每次扩展得到独立分支，不会修改原来的前缀。
  // 接下来最关键的问题是：应该怎样检查整段运动，才能避免采样之间漏碰撞？
  // 一个运动，就是由 三个因素来确定的：弧长、曲率、和旋转角度！
  auto const extend = [&](Candidate candidate, Motion const& motion)
  {
    candidate.motions.push_back(motion);
    Pose const from = candidate.poses.back();
    double const angle = motion.rotation + motion.distance * motion.curvature;
    // 任何车身点的路程 <= |ds| + radius*|dtheta|。
    //          >>> 任何车身点的运动路程都不超过  bound
    // 这个上界同时覆盖：
    //    - 平移造成的位移。
    //    - 旋转造成的车身角点运动。
    //   所以原地旋转时，即使 distance == 0，仍然会根据车身尺寸和旋转角度安排采样。
    // bound 通常不等于某个车身点的真实运动路程。但更准确地说，它是一个
    // 有几何依据的保守上界，并不是依赖小角度成立的近似。
    /*
    为什么它通常大于真实路程
      因为真实速度是两个向量相加后的模长：
        > 虽然只计算了区间中点的净空，但在这个小区间内，车身任何点都不可能移动
        > 超过 half_motion_bound，因此净空最多也只会减少这么多。
    */
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
      // 检查中点，再扣除运动误差
      // 对每个采样区间，代码取它的中点位姿：
      // advance 这里的位姿推进和整车距离怎么计算
      Pose const midpoint = advance(from, motion, (i + 0.5) / steps);
      // 计算中点的整车净空：
      double const clearance = footprint_clearance(grid,
                                                   midpoint,
                                                   footprint,
                                                   config.clearance_cap,
                                                   config.occupied_threshold);
      // 距离函数的 Lipschitz 界：中点距离减最大位移，覆盖整个采样区间，
      // 包括平移跨薄障碍、原地旋转角点扫过障碍的情况。
      // 然后得到整个区间的净空下界：
      // [
      // half_motion_bound=\frac{B}{2N}
      // ]
      // 为什么这样能覆盖整个区间？
      // 因为从区间中点运动到区间内任意时刻，任何车身点的位移都不超过 (B/
      // (2N))。而车身与障碍的最短距离，减少量不会超过这个位移上界。
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

  // record：区分“安全前缀”和“完整脱困方案”
  auto const record = [&](Candidate candidate, bool terminal)
  {
    Pose const& end = candidate.poses.back();
    // 首先计算相对最初位姿的位移：
    Vec const offset = end.position - start.position;
    Vec const forward = direction(start);
    Vec const left(-forward.y(), forward.x());
    // 然后以最初朝向为基准计算净后退进度：
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
    lateral_boundary_viz::candidate(index, candidate, footprint);
    if(candidate.feasible &&
       (!result.selected ||
        candidate.score > result.candidates[*result.selected].score))
    {
      result.selected = index;
    }
    result.candidates.push_back(std::move(candidate));
  };

  /*
  重新计算 prefix 末端的边界
  生成曲率集合
  对每个曲率：
      extend(prefix, 后退动作)
      record(..., true)
  每个后退动作的有符号弧长都是：

  -config.reverse_length

  所以后退长度固定，但曲率可变。

  这里每次从当前前缀末端重新计算边界，非常关键：前进或旋转之后，机器人面
  对的障碍分布已经变化，不能一直使用起点的边界建议。
  */
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

  /*
  try_rotations(prefix)：

  对每个预设旋转角：
      rotated = extend(prefix, 原地旋转)
      record(rotated, false)

      如果旋转安全：
          try_reverse(rotated)

  旋转本身只是调整姿态，所以 terminal == false。

  前进搜索：

  对每个预设前进距离：
      计算相应边界建议

      对每个候选曲率：
          forward = extend(root, 前进动作)
          record(forward, false)

          如果前进安全：
              try_reverse(forward)
              保存 forward 前缀

  注意，每个前进前缀都从 root 出发，不会把不同的前进长度连续叠加起来。

  最后，代码把所有安全前进前缀按 min_clearance 从大到小排序，只取前
  forward_beam_width 个，继续尝试：

  前进 → 原地旋转 → 后退

  默认宽度是 3。
  */
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

  // 首先尝试后退，如果后退成功，就不需要前进或者旋转了
  try_reverse(root);
  // 已有完整后退轨迹时优先执行，不引入不必要的前进/旋转。
  // 如果后退失败，那么就尝试其它的：前进、旋转
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
  lateral_boundary_viz::finish(result, footprint);
  return result;
}
} // namespace local_escape

/*
  十二、怎样理解这个函数的能力边界

  读完实现，最值得保留的是下面几条：

  - **边界信息负责引导，完整车身检查负责确认安全。**左右射线和小角度拟合
    只是近似，不等于车身扫掠区域。

  - **安全判断覆盖采样区间。**它通过“中点净空减最大位移”建立保守下界，避
    免仅凭离散点净空来判断。

  - **完整后退优先。**有直接后退解时，不会为了更高分而额外尝试前进或旋
    转。

  - **no_path 只表示当前候选集合没有找到可行方案。**有限曲率、固定后退长
    度、有限动作结构和前缀筛选，都可能遗漏实际存在的路径。

  - **输出是几何轨迹。**代码没有进行速度、加速度或执行时间规划，也没有处
    理运动中的动态障碍；接口注释明确要求执行侧做速度规划并在执行中重新规
    划。

  仓库中的对应测试 (tests/planning/local_escape_planner_test.cc:124)也围
  绕这些设计展开：开阔空间直接后退、侧后方障碍下的曲线后退、先前进再恢
  复、旋转中途角点扫障碍，以及完全封闭时拒绝把安全前缀当作成功。
*/

/*
• 你的疑问有道理。这段 plan() 使用 lambda 有明确的便利，但几个 lambda 的
  职责和调用关系已经比较复杂，可读性上存在取舍。

  关键不在于 lambda 的数量，而在于：它们是否足够局部、依赖是否清楚，以及
  读者是否容易理解它们之间的关系。

  1. 为什么作者会这样写

  plan() 中主要定义了四个 lambda：

  extend(candidate, motion)   // 扩展轨迹并检查安全性
  record(candidate, terminal) // 计算评分、记录并更新最优候选
  try_reverse(prefix)        // 尝试不同曲率的后退
  try_rotations(prefix)      // 尝试旋转，然后后退

  它们都只服务于这一次规划，而且共享大量上下文：

  grid
  start
  footprint
  config
  result
  radius
  step_bound

  使用：

  auto const extend = [&](Candidate candidate, Motion const& motion)
  {
      // 可以直接访问上述局部变量
  };

  就不必给每个辅助函数反复传递这些参数。

  如果直接改成普通函数，extend 的调用可能变成：

  extend(candidate, motion,
         grid, footprint, config, radius, step_bound);

  record 又需要另一组参数：

  record(candidate, terminal, start, config, result);

  因此，lambda 在这里起到了函数内部辅助函数的作用，把相关逻辑集中在一
  起，同时减少参数传递。

  这也让最后的搜索流程相对直观：

  try_reverse(root);

  if (!result.selected)
  {
      try_rotations(root);
      // 搜索前进及其后续动作……
  }

  2. 这里的 lambda 是闭包，不只是匿名函数

  理解这里的写法，要关注捕获列表：

  [&]

  它允许 lambda 按引用捕获用到的外部局部变量。

  例如：

  auto const record = [&](Candidate candidate, bool terminal)
  {
      // ...
      result.candidates.push_back(std::move(candidate));
  };

  result 没有出现在参数列表中，但 record 可以直接修改它。

  因此，实际依赖更接近：

  显式参数：candidate、terminal
  隐式上下文：start、config、result

  这种写法很方便，但也正是阅读成本的来源：仅看参数列表，无法知道这个操作
  读取了什么、修改了什么。

  另外：

  auto const record = ...

  中的 const 并不意味着它没有副作用。它仍然可以通过引用修改外部的
  result。

  3. 当前实现中，哪些用法比较合适

  用于排序的小 lambda 非常合适：

  [](Candidate const& a, Candidate const& b)
  {
      return a.min_clearance > b.min_clearance;
  }

  它很短，参数明确，没有隐藏状态，逻辑也正好属于排序这一处。单独提取命名
  函数未必更清楚。

  try_reverse 也有使用局部 lambda 的理由：

  auto const try_reverse = [&](Candidate const& prefix)
  {
      auto const boundaries = profile(...);

      for (double curvature : curvatures(boundaries, config))
      {
          record(extend(prefix, {...}), true);
      }
  };

  它表达了一个明确的局部步骤：“从这个前缀末端尝试后退”。

  对于这种只在当前算法中使用的短小流程，lambda 很自然。

  4. 哪些地方已经开始影响可读性

  主要是下面三个方面。

  首先，extend 承担的职责比较重。

  它包括：

  - 运动分段。
  - 位姿积分调用。
  - 整车净空计算。
  - 连续区间安全下界判断。
  - 失败信息记录。
  - 轨迹端点和最小净空更新。

  这已经是一个独立的核心算法单元。把它放在 plan() 内部，意味着读者还没看
  到整体搜索流程，就要先读几十行安全检查实现。

  它值得成为一个有明确接口的命名函数或成员函数。

  其次，lambda 之间形成了多层依赖：

  try_rotations
      ├─ extend
      ├─ record
      └─ try_reverse
             ├─ extend
             └─ record

  这些依赖通过 [&] 隐式捕获。

  例如，调用：

  try_rotations(root);

  实际上不仅尝试旋转，还会尝试后退，并且向 result 写入多个候选、更新最优
  解。

  这种副作用并没有从函数名和参数中完全显露出来。名字改成类似
  search_rotation_then_reverse，会更准确。

  最后，record 的名字低估了它的职责：

  计算后退进度
  判断可行性
  设置原因
  计算评分
  更新最优候选
  保存候选

  它不仅是“记录”。如果命名为：

  evaluate_and_record(...)

  读者会更容易预期它的行为。

  这些问题并不是 lambda 特有的；普通函数如果命名模糊、职责过多，也会出
  现。但隐式捕获会让问题更明显。

  5. 会不会有性能或生命周期问题

  就当前写法而言，lambda 本身通常不是性能问题。

  这里没有使用 std::function，lambda 的具体类型在编译期已知，也不要求为
  了保存闭包而进行堆分配。编译器有机会像处理普通函数一样对其内联优化。

  [&] 捕获的对象也都在 plan() 执行期间存活，而这些 lambda 没有被返回或交
  给异步任务，因此当前用法不存在“离开函数后还引用局部变量”的问题。

  更值得关注的性能点是候选复制：

  auto const extend = [&](Candidate candidate, ...)

  这里按值接收 Candidate。从已有前缀扩展分支时，会复制其中的 poses、
  motions 等数据。

  这是为了让不同搜索分支独立，具有合理性。即使把 lambda 改成普通函数，只
  要参数仍然按值传递，这些复制仍然存在。

  所以不能简单把可能的开销归因于 lambda。

  6. 对这份代码，我会怎样调整

  对于目前这个实验模块，我倾向于做适度重构：

   部分                          建议
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   排序比较 lambda               保留
  ────────────────────────────  ────────────────────────────────────────
   extend                        提取为命名函数，明确输入依赖
  ────────────────────────────  ────────────────────────────────────────
   record                        至少重命名为 evaluate_and_record，必要
                                 时提取
  ────────────────────────────  ────────────────────────────────────────
   try_reverse、try_rotations    可以继续作为局部流程，也可以随搜索逻辑
                                 一起封装
  ────────────────────────────  ────────────────────────────────────────
   [&]                           对保留的较复杂 lambda，考虑显式列出捕
                                 获项

  例如，record 可以先改成：

  auto const evaluate_and_record =
      [&result, &start, &config](Candidate candidate, bool terminal)
  {
      // ...
  };

  这不会改变算法，但能让读者立即看出：

  > 它依赖起点和配置，并且会访问外部结果。

  如果将来搜索策略继续增长，再引入一个 .cc 内部的规划上下文类，把共享数
  据和操作放在一起：

  class PlanningContext
  {
  public:
      Result run();

  private:
      Candidate extend(Candidate candidate, Motion const& motion);
      void evaluate_and_record(Candidate candidate, bool terminal);
      void try_reverse(Candidate const& prefix);
      void try_rotations(Candidate const& prefix);

      // 地图、轮廓、配置、结果等共享上下文
  };

  这样既避免长参数列表，也能把大段实现移出主流程。不过，以当前规模来说，
  没有必要仅仅为了消除 lambda 就引入一个类。

  我的判断是：这段代码使用 lambda 的动机合理，当前同步调用方式也没有明显
  的捕获生命周期问题；但 extend 和 record 已经适合进一步拆分。最值得改善
  的是核心算法的独立性、操作命名和隐式依赖，而不是机械地减少 lambda 数
  量。

*/
