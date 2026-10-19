#include "local_escape.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>


namespace algorithms
{


namespace
{
using SearchClock = std::chrono::steady_clock;
struct TimingEntry
{
  double milliseconds = 0.0;
  std::size_t calls = 0;
};
struct SearchTiming
{
  static constexpr std::array<char const*, 6> phase_names{
      "prepare/validate/initial boundary",
      "start clearance",
      "direct reverse search",
      "rotate + reverse search",
      "forward + reverse search",
      "forward + rotate + reverse beam"};
  static constexpr std::array<char const*, 4> detail_names{
      "compute_boundary_profile",
      "footprint_clearance",
      "extend",
      "evaluate_and_record"};
  std::array<TimingEntry, 6> phases{};
  std::array<TimingEntry, 4> details{};
  std::size_t cell_distance_calls = 0;
  SearchClock::time_point begin = SearchClock::now();
  SearchClock::time_point phase_begin = begin;
  std::size_t phase_index = 0;

  void
  finish_phase(SearchClock::time_point now)
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    phases[phase_index].milliseconds +=
        std::chrono::duration<double, std::milli>(now - phase_begin).count();

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    ++phases[phase_index].calls;
    phase_begin = now;
  }
  void
  phase(std::size_t index)
  {
    finish_phase(SearchClock::now());
    phase_index = index;
  }
  ~SearchTiming()
  {
    auto const end = SearchClock::now();
    finish_phase(end);
    double const total =
        std::chrono::duration<double, std::milli>(end - begin).count();
    std::fprintf(stderr, "[search timing] plan total: %.3f ms\n", total);
    // 阶段互不重叠，可相加；detail 是包含子调用的累计耗时，不可相加。
    for(std::size_t i = 0; i < phases.size(); ++i)
    {
      std::fprintf(
          stderr,
          "[search timing] phase %-38s %.3f ms%s\n",
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
          phase_names[i],
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
          phases[i].milliseconds,
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
          (phases[i].calls != 0U) ? "" : " (skipped)");
    }
    for(std::size_t i = 0; i < details.size(); ++i)
    {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
      auto const& d = details[i];
      std::fprintf(
          stderr,
          "[search timing] detail %-24s %.3f ms, calls=%zu, avg=%.6f "
          "ms (inclusive)\n",
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
          detail_names[i],
          d.milliseconds,
          d.calls,
          (d.calls != 0U) ? d.milliseconds / d.calls : 0.0);
    }
    std::fprintf(stderr,
                 "[search timing] cell_distance calls=%zu\n",
                 cell_distance_calls);
  }
};

class DetailTimer
{
public:
  explicit DetailTimer(TimingEntry& entry)
    : entry_(entry)
    , begin_(SearchClock::now())
  {}
  ~DetailTimer()
  {
    entry_.milliseconds +=
        std::chrono::duration<double, std::milli>(SearchClock::now() - begin_)
            .count();
    ++entry_.calls;
  }

  DetailTimer(DetailTimer const&) = delete;
  DetailTimer&
  operator=(DetailTimer const&) = delete;
  DetailTimer(DetailTimer&&) = delete;
  DetailTimer&
  operator=(DetailTimer&&) = delete;

private:
  TimingEntry& entry_;
  SearchClock::time_point begin_;
};
} // namespace

namespace
{
constexpr double epsilon = 1e-10;

// 单次规划共享的只读输入和派生参数，不持有搜索结果或计时状态。
// 引用及 span 借用调用方的数据，生命周期不超过 plan()。
struct Context
{
  GridMap<std::uint8_t> const& grid;
  Pose const& start;
  std::span<Eigen::Vector2d const> footprint;
  Config const& config;
  double radius;
  double step_bound;

  Context(GridMap<std::uint8_t> const& grid,
          Pose const& start,
          std::span<Eigen::Vector2d const> footprint,
          Config const& config,
          double radius,
          double step_bound)
    : grid(grid)
    , start(start)
    , footprint(footprint)
    , config(config)
    , radius(radius)
    , step_bound(step_bound)
  {}

  Context(Context const&) = delete;
  Context&
  operator=(Context const&) = delete;
  Context(Context&&) = delete;
  Context&
  operator=(Context&&) = delete;
};

Context
make_context(GridMap<std::uint8_t> const& grid,
             Pose const& start,
             std::span<Eigen::Vector2d const> footprint,
             Config const& config)
{
  double radius = 0.0;
  for(auto const& p : footprint)
  {
    radius = std::max(radius, p.norm());
  }
  // double const step_bound =
  //     std::min({config.max_corner_step,
  //               grid.geometry().resolution * 0.5,
  //               (config.clearance_cap - config.safety_margin) * 0.5});
  double const step_bound =
      std::min({config.max_corner_step,
                (config.clearance_cap - config.safety_margin) * 0.5});
  return {grid, start, footprint, config, radius, step_bound};
}


Eigen::Vector2d
direction(Pose const& pose)
{
  return {std::cos(pose.yaw), std::sin(pose.yaw)};
}


/*
单词	表示
width	宽度，通常是一个非负标量
extent	占据范围，可以包含最小值和最大值
offset	相对于参考位置的偏移量
boundary	边界位置或边界几何
*/
// extent：范围、延伸程度，表示一个物体在某个方向上占据的空间范围。
std::pair<double, double>
lateral_extent(std::span<Eigen::Vector2d const> footprint, double x)
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


bool
inside(Eigen::Vector2d const& p, std::span<Eigen::Vector2d> polygon)
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

double
point_segment_distance(Eigen::Vector2d const& p,
                       Eigen::Vector2d const& a,
                       Eigen::Vector2d const& b)
{
  Eigen::Vector2d const edge = b - a;
  double const t =
      edge.squaredNorm() == 0.0
          ? 0.0
          : std::clamp((p - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
  return (p - a - t * edge).norm();
}

double
cross(Eigen::Vector2d const& a, Eigen::Vector2d const& b)
{
  return a.x() * b.y() - a.y() * b.x();
}
bool
segments_intersect(Eigen::Vector2d const& a,
                   Eigen::Vector2d const& b,
                   Eigen::Vector2d const& c,
                   Eigen::Vector2d const& d)
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
cell_distance(std::span<Eigen::Vector2d> polygon,
              Eigen::Vector2d const& lo,
              Eigen::Vector2d const& hi)
{
  std::array<Eigen::Vector2d, 4> const corners{lo,
                                               Eigen::Vector2d(hi.x(), lo.y()),
                                               hi,
                                               Eigen::Vector2d(lo.x(), hi.y())};
  if(inside(lo, polygon))
  {
    return 0.0; // 整个障碍格位于 footprint 内部
  }
  double result = std::numeric_limits<double>::infinity();
  for(std::size_t i = 0; i < polygon.size(); ++i)
  {
    auto const& a = polygon[i];
    auto const& b = polygon[(i + 1) % polygon.size()];
    Eigen::Vector2d const offset = (lo - a).cwiseMax(a - hi).cwiseMax(0.0);
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
// 传入 travel 通常表示行程，而你这里真正使用的是沿初始车身 X 轴的有符号纵向位移。
BoundaryProfile
compute_boundary_profile(Context const& ctx,
                         Pose const& pose,
                         double travel,
                         SearchTiming& timing)
{
  DetailTimer const detail_timer(timing.details[0]);
  // 这里的 ctx.footprint 按前文约定，使用车身坐标系。并非是在 gridmap 中的坐标！
  // 纵向查询范围，机器人自身坐标系
  double xmin = ctx.footprint.front().x();
  double xmax = xmin;
  for(auto const& p : ctx.footprint)
  {
    xmin = std::min(xmin, p.x());
    xmax = std::max(xmax, p.x());
  }
  BoundaryProfile result;
  // 当前车身及运动后车身所在区域都查询，不能只查询当前车长。
  // 这个范围只按直线平移构造，不等于旋转运动的完整扫掠包络。
  double const begin = xmin + std::min(0.0, travel);
  double const end = xmax + std::max(0.0, travel);

  double const intervals = std::max(1.0, std::ceil((end - begin) / 0.1));
  if(!std::isfinite(intervals) || intervals > 100000)
  {
    throw std::invalid_argument("Local escape: boundary range is too large");
  }
  std::size_t const count = static_cast<std::size_t>(intervals) + 1;
  result.longitudinal_samples.reserve(count);

  for(std::size_t i = 0; i < count; ++i)
  {
    result.longitudinal_samples.push_back(begin +
                                          (end - begin) * i / (count - 1));
  }

  result.boundary =
      latter_boundary::build_lateral_boundary(ctx.grid,
                                              pose.position,
                                              direction(pose),
                                              result.longitudinal_samples,
                                              ctx.config.occupied_threshold);

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
    auto const [low, high] = lateral_extent(ctx.footprint, body_x);

    // 从障碍距离中扣除车身，得到左右净空
    // 左右障碍物的距离 - 车身的宽度 - 安全余量
    double const left_clearance =
        result.boundary.left_width[i] - high - ctx.config.safety_margin;
    double const right_clearance =
        result.boundary.right_width[i] + low - ctx.config.safety_margin;

    result.left_clearance.push_back(left_clearance);
    result.right_clearance.push_back(right_clearance);


    // 左右斜率：净空沿车身纵向如何变化
    //         实际上在计算左右净空关于纵向坐标的线性拟合斜率。
    // 这里做的是一个最小二乘法拟合
    // 中心化后的纵向坐标（Centered Longitudinal Coordinate）。
    double const centered_s = s - mean_s;
    cov_left += centered_s * left_clearance;
    cov_right += centered_s * right_clearance;
    variance += centered_s * centered_s;
    // 也就是只选择预计运动完成后，车身占据的纵向范围
    // 这也说明该曲率拟合主要针对运动结束时的车身位置，不是逐时刻的轨迹优化
    // 拟合运动结束时的车身侧移
    // 只用运动后车身所在区间拟合曲率
    // travel： 旅行，行程、（有符号行驶距离），这个 travel 表示的是后退的距离，不是自己一开始理解的向后延长的边界
    //
    if(s >= xmin + travel && s <= xmax + travel)
    {
      // 小角度下车身截面的侧移约 k*(travel^2/2 + travel*body_x)
      // 包含车头外摆与车尾侧移，且使用前后各截面的边界变化
      // desired： 这个截面希望向哪边移动多少，尽可能的向净空多的一侧移动
      // desired	期望值、目标值	希望当前车身截面横向移动多少
      // desired：期望侧移量，对于一段位移，期望它的侧移量，不要过大
      double const desired_lateral_shift =
          std::clamp(0.5 * (left_clearance - right_clearance), -0.25, 0.25);
      // coefficient	系数	描述曲率 \(k\) 对截面侧移的影响
      // 曲率对应的侧移系数  这个变量来自机器人常曲率运动模型的小角度近似。
      // 它的物理意义是：单位曲率变化对该车身截面横向位移的影响系数。
      double const lateral_shift_coefficient =
          0.5 * travel * travel + travel * body_x;
      // weight	权重	当前截面的侧移目标有多重要
      /*
          weight 表示该采样截面在优化中的重要程度。
          这里的设计原则是：
          哪一侧离障碍物越近，该截面在曲率拟合中就越重要。
      */
      double const clearance_weight =
          1.0 /
          (0.05 + std::max(0.0, std::min(left_clearance, right_clearance)));
      numerator +=
          clearance_weight * lateral_shift_coefficient * desired_lateral_shift;
      denominator += clearance_weight * lateral_shift_coefficient *
                     lateral_shift_coefficient;
    }
  }
  result.left_slope = cov_left / variance;
  result.right_slope = cov_right / variance;
  result.preferred_curvature =
      std::clamp(denominator > epsilon ? numerator / denominator : 0.0,
                 -ctx.config.max_curvature,
                 ctx.config.max_curvature);
  return result;
}


std::vector<Eigen::Vector2d>
world_footprint(Pose const& pose, std::span<Eigen::Vector2d const> footprint)
{
  Eigen::Vector2d const forward = direction(pose);
  Eigen::Vector2d const left(-forward.y(), forward.x());
  std::vector<Eigen::Vector2d> result;
  result.reserve(footprint.size());
  for(auto const& p : footprint)
  {
    result.emplace_back(pose.position + p.x() * forward + p.y() * left);
  }
  return result;
}



double
footprint_clearance(Context const& ctx, Pose const& pose, SearchTiming& timing)
{
  DetailTimer const detail_timer(timing.details[1]);
  // 将局部 ctx.footprint 转换到全局 pose
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
      ds * sinc *
      Eigen::Vector2d(std::cos(start.yaw + half), std::sin(start.yaw + half));
  result.yaw += angle;
  return result;
}


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
  double const half_motion_bound = bound / steps * 0.5;

  for(std::size_t i = 0; i < steps; ++i)
  {
    // 运动 i+0.5 个 step 之后的位资
    Pose const midpoint = advance(from, motion, (i + 0.5) / steps);
    // 计算当前的净空
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
  std::ranges::sort(values);
  auto const duplicates = std::ranges::unique(
      values, [](double a, double b) { return std::abs(a - b) < epsilon; });
  values.erase(duplicates.begin(), duplicates.end());

  //  第二次排序：优先尝试更接近建议的曲率
  std::ranges::stable_sort(
      values,
      [preferred](double a, double b)
      { return std::abs(a - preferred) < std::abs(b - preferred); });
  return values;
}


void
evaluate_and_record(Context const& ctx,
                    Candidate candidate,
                    bool terminal,
                    Result& result,
                    SearchTiming& timing)
{
  DetailTimer const detail_timer(timing.details[3]);
  Pose const& end = candidate.poses.back();
  // 首先计算相对最初位姿的位移：
  Eigen::Vector2d const offset = end.position - ctx.start.position;
  Eigen::Vector2d const forward = direction(ctx.start);
  Eigen::Vector2d const left(-forward.y(), forward.x());

  candidate.reverse_progress = -offset.dot(forward);
  // feasible adj.可行的
  candidate.feasible =
      terminal && !candidate.rejected_pose &&
      candidate.reverse_progress >= ctx.config.min_reverse_progress;

  if(!candidate.rejected_pose)
  {
    if(!terminal)
    {
      candidate.reason = "reposition_prefix";
    }
    else if(candidate.feasible)
    {
      candidate.reason = "feasible";
    }
    else
    {
      candidate.reason = "insufficient_reverse_progress";
    }
  }

  double forward_distance = 0.0;
  double turning = 0.0;
  double curvature_effort = 0.0;

  // 当前候选轨迹中所有的动作
  for(auto const& motion : candidate.motions)
  {
    forward_distance += std::max(0.0, motion.distance);
    turning += std::abs(motion.rotation);
    curvature_effort += std::abs(motion.distance * motion.curvature);
  }

  // 整个候选路径的评估
  candidate.score = 4.0 * candidate.reverse_progress +
                    2.0 * candidate.min_clearance - forward_distance -
                    0.25 * turning - 0.15 * curvature_effort -
                    0.4 * std::abs(offset.dot(left)) -
                    0.2 * std::abs(end.yaw - ctx.start.yaw);

  std::size_t const index = result.candidates.size();
  if(candidate.feasible &&
     (!result.selected ||
      candidate.score > result.candidates[*result.selected].score))
  {
    result.selected = index;
  }
  result.candidates.push_back(std::move(candidate));
}


void
try_reverse(Context const& ctx,
            Candidate const& prefix,
            Result& result,
            SearchTiming& timing)
{
  auto const boundaries = compute_boundary_profile(ctx,
                                                   prefix.poses.back(),
                                                   -ctx.config.reverse_length,
                                                   timing);
  for(double curvature : curvatures(boundaries, ctx.config))
  {
    evaluate_and_record(ctx,
                        extend(ctx,
                               prefix,
                               {-ctx.config.reverse_length, curvature, 0.0},
                               timing),
                        true,
                        result,
                        timing);
  }
}


void
try_rotations(Context const& ctx,
              Candidate const& prefix,
              Result& result,
              SearchTiming& timing)
{
  if(!ctx.config.allow_in_place_rotation)
  {
    return;
  }
  for(double angle : ctx.config.rotation_angles)
  {
    auto rotated = extend(ctx, prefix, {0.0, 0.0, angle}, timing);
    evaluate_and_record(ctx, rotated, true, result, timing);
    if(!rotated.rejected_pose)
    {
      try_reverse(ctx, rotated, result, timing);
    }
  }
}


void
validate(algorithms::GridMap<std::uint8_t> const& grid,
         Pose const& start,
         std::span<Eigen::Vector2d const> footprint,
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
    Eigen::Vector2d const& a = footprint[i];
    Eigen::Vector2d const& b = footprint[(i + 1) % footprint.size()];
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

Result
search(Context const& ctx, SearchTiming& timing)
{
  Result result;
  // 这是初始位姿下，面向后退动作的左右边界信息。它主要用于解释环境和引导候选，不能单独证明轨迹安全。
  result.boundary = compute_boundary_profile(ctx,
                                             ctx.start,
                                             -ctx.config.reverse_length,
                                             timing);
  timing.phase(1);
  // 起点安全检查
  double const initial_clearance = footprint_clearance(ctx, ctx.start, timing);
  // 这里要求严格大于安全余量。也就是说，即使起点尚未发生实际碰撞，只要安全
  // 余量不足，也返回 start_blocked。
  if(initial_clearance <= ctx.config.safety_margin + epsilon)
  {
    result.status = Status::start_blocked;
    return result;
  }

  // 标识整个运动的起点
  Candidate root;
  root.poses.push_back(ctx.start);
  root.min_clearance = initial_clearance;

  timing.phase(2);
  try_reverse(ctx, root, result, timing);
  if(!result.selected)
  {
    timing.phase(3);
    try_rotations(ctx, root, result, timing);
    timing.phase(4);
    std::vector<Candidate> forward_prefixes;
    for(double length : ctx.config.forward_lengths)
    {
      auto const boundaries =
          compute_boundary_profile(ctx, ctx.start, length, timing);
      for(double curvature : curvatures(boundaries, ctx.config))
      {
        auto forward = extend(ctx, root, {length, curvature, 0.0}, timing);
        evaluate_and_record(ctx, forward, false, result, timing);
        if(!forward.rejected_pose)
        {
          try_reverse(ctx, forward, result, timing);
          forward_prefixes.push_back(std::move(forward));
        }
      }
    }
    timing.phase(5);
    // 有限宽度搜索前进 + 原地旋转 + 后退；每次重新计算当地左右边界。
    std::ranges::stable_sort(forward_prefixes,
                     [](Candidate const& a, Candidate const& b)
                     { return a.min_clearance > b.min_clearance; });
    for(std::size_t i = 0;
        i < std::min(ctx.config.forward_beam_width, forward_prefixes.size());
        ++i)
    {
      try_rotations(ctx, forward_prefixes[i], result, timing);
    }
  }
  result.status = result.selected ? Status::success : Status::no_path;
  return result;
}

} // namespace

Result
plan(algorithms::GridMap<std::uint8_t> const& grid,
     Pose const& start,
     std::span<Eigen::Vector2d const> footprint,
     Config const& config)
{
  SearchTiming timing;
  if(footprint.size() > 1 && footprint.front() == footprint.back())
  {
    footprint = footprint.first(footprint.size() - 1);
  }
  validate(grid, start, footprint, config);

  Context const ctx = make_context(grid, start, footprint, config);
  return search(ctx, timing);
}

} // namespace algorithms
