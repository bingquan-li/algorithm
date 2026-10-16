#include "reference_line.h"

#include <cmath>

#include "debug.h"


#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>
#include <Eigen/Core>

namespace
{

constexpr double kEpsilon = 1e-6;

double
normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct BoundaryVertex
{
  Eigen::Vector2d xy;
  std::size_t sample;
  bool left;
};

} // namespace

namespace reference_line
{

bool
ReferenceLine::xy_to_sl(Vec2d const& xy_point,
                        SLPoint* const sl_point,
                        double warm_start_s) const
{
  if(reference_points_.size() < 2)
  {
    REFERENCE_ERROR_LOG("xy:[",
                        xy_point.x,
                        ",",
                        xy_point.y,
                        "] to sl failed! reference_points size < 2:[",
                        reference_points_.size(),
                        "]");
    return false;
  }
  double best_distance{std::numeric_limits<double>::max()};
  bool found{false};
  for(std::size_t idx = 0; idx < reference_points_.size() - 1; ++idx)
  {
    ReferencePoint start = reference_points_[idx];
    ReferencePoint end = reference_points_[idx + 1];
    double const dx = end.xy().x - start.xy().x;
    double const dy = end.xy().y - start.xy().y;
    double length_squared = dx * dx + dy * dy;
    if(length_squared < kEpsilon)
    {
      continue;
    }

    double ratio = std::clamp(
        ((xy_point.x - start.xy().x) * dx + (xy_point.y - start.xy().y) * dy) /
            length_squared,
        0.0,
        1.0);
    for(int it = 0; it++ < 8;)
    {
      // 采样得到的是 笛卡尔坐标系下的
      auto const sample = sample_reference(idx, ratio);
      double const gradient = (sample.x - xy_point.x) * sample.dx +
                              (sample.y - xy_point.y) * sample.dy;
      double const hessian = sample.dx * sample.dx + sample.dy * sample.dy +
                             (sample.x - xy_point.x) * sample.ddx +
                             (sample.y - xy_point.y) * sample.ddy;
      if(hessian <= 1e-12)
      {
        break;
      }
      double const next = std::clamp(ratio - gradient / hessian, 0.0, 1.0);
      if(std::abs(next - ratio) < 1e-9)
      {
        break;
      }
      ratio = next;
    }
    auto projected = sample_reference(idx, ratio);
    double distance_squared = std::pow(xy_point.x - projected.x, 2) +
                              std::pow(xy_point.y - projected.y, 2);
    for(double const endpoint : {0.0, 1.0})
    {
      auto const sample = sample_reference(idx, endpoint);
      double const distance = std::pow(xy_point.x - sample.x, 2) +
                              std::pow(xy_point.y - sample.y, 2);
      if(distance < distance_squared)
      {
        ratio = endpoint;
        projected = sample;
        distance_squared = distance;
      }
    }
    double const projected_s =
        start.sl().s + ratio * (end.sl().s - start.sl().s);
    // todo 这里的 minimum s 是否有必要？
    // if(std::isfinite(minimum_s) && projected_s < minimum_s)
    // {
    //   continue;
    // }
    double const tangent_length = std::hypot(projected.dx, projected.dy);
    if(distance_squared < best_distance && tangent_length > kEpsilon)
    {
      best_distance = distance_squared;
      sl_point->s = projected_s;
      sl_point->l = (projected.dx * (xy_point.y - projected.y) -
                     projected.dy * (xy_point.x - projected.x)) /
                    tangent_length;
      found = true;
    }
  }
  return found;
}

bool
ReferenceLine::sl_to_xy(SLPoint const& sl_point, Vec2d* const xy_point) const
{
  if(reference_points_.size() < 2)
  {
    return false;
  }

  auto const matched_point = get_reference_point(sl_point.s);
  xy_point->x =
      (matched_point.xy().x - std::sin(matched_point.theta()) * sl_point.l);
  xy_point->y =
      (matched_point.xy().y + std::cos(matched_point.theta()) * sl_point.l);
  return true;
}

ReferencePoint
ReferenceLine::get_reference_point(double const s) const
{
  if(reference_points_.empty())
  {
    return ReferencePoint{};
  }

  if(s <= reference_points_.front().sl().s)
  {
    return reference_points_.front();
  }
  if(s >= reference_points_.back().sl().s)
  {
    return reference_points_.back();
  }

  auto interpolate_index = get_index_from_s(s);

  size_t index = interpolate_index.id;
  size_t next_index = index + 1;
  if(next_index >= reference_points_.size())
  {
    next_index = reference_points_.size() - 1;
  }

  auto const& p0 = reference_points_[index];
  auto const& p1 = reference_points_[next_index];

  double const s0 = p0.sl().s;
  double const s1 = p1.sl().s;
  return interpolate_with_matched_index(p0, s0, p1, s1, interpolate_index);
}

InterpolatedIndex
ReferenceLine::get_index_from_s(double s) const
{
  if(reference_points_.empty() || s <= reference_points_.front().sl().s)
  {
    return {0, 0.0};
  }
  if(s >= reference_points_.back().sl().s)
  {
    return {static_cast<int>(reference_points_.size()) - 1, 0.0};
  }

  // accumulated_s_ 保存完整路径；查询索引必须属于当前截断后的参考点。
  auto upper = std::upper_bound(reference_points_.begin(),
                                reference_points_.end(),
                                s,
                                [](double query_s, ReferencePoint const& point)
                                { return query_s < point.sl().s; });

  int const right_index =
      static_cast<int>(std::distance(reference_points_.begin(), upper));
  int const left_index = right_index - 1;

  return {left_index, s - reference_points_[left_index].sl().s};
}

ReferencePoint
ReferenceLine::interpolate_with_matched_index(
    ReferencePoint const& p0,
    double const s0,
    ReferencePoint const& p1,
    double const s1,
    InterpolatedIndex const& index) const
{
  double const segment_length = s1 - s0;
  if(segment_length <= kEpsilon)
  {
    return p0;
  }

  double const t = std::clamp(index.offset / segment_length, 0.0, 1.0);
  double const matched_s = s0 + t * segment_length;

  double const x = p0.xy().x + t * (p1.xy().x - p0.xy().x);
  double const y = p0.xy().y + t * (p1.xy().y - p0.xy().y);

  double const theta = normalize_angle(
      p0.theta() + t * normalize_angle(p1.theta() - p0.theta()));
  double const kappa = p0.kappa() + t * (p1.kappa() - p0.kappa());
  double const dkappa = p0.dkappa() + t * (p1.dkappa() - p0.dkappa());

  // todo 这里没有构造 tangent
  return ReferencePoint(Vec2d{x, y},
                        theta,
                        SLPoint{matched_s, 0.0},
                        kappa,
                        dkappa);
}

bool
ReferenceLine::update_reference_line(Vec2d const& cur_pose)
{
  if(!truncated_)
  {
    return true;
  }

  SLPoint cur_sl;
  if(!xy_to_sl(cur_pose, &cur_sl))
  {
    return false;
  }
  truncated_ = false;
  double cur_s = std::max(0.0, cur_sl.s - 1.0);
  auto const it =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), cur_s);

  std::size_t const idx = std::distance(accumulated_s_.begin(), it);
  for(std::size_t i = idx; i < accumulated_s_.size(); ++i)
  {
    if(accumulated_s_[i] - cur_s > config_->max_reference_line_length)
    {
      truncated_ = true;
      truncated_index_ = i;
      break;
    }
  }
  if(!truncated_)
  {
    truncated_index_ = accumulated_s_.size();
  }

  if(!build_reference_points())
  {
    REFERENCE_DEBUG_LOG(
        "build reference failed! build reference points failed!");
    valided_ = false;
    return false;
  }

  build_lateral_boundary();

  if(!debug())
  {
    // 调试失败通常不阻止主流程
    REFERENCE_DEBUG_LOG("reference line built, but debug generation failed");
  }
  return true;
}



bool
ReferenceLine::create_new_line(std::vector<Eigen::Vector2d> const& global_path,
                               Eigen::Vector2d const& dir,
                               double resample_spacing,
                               double min_length,
                               bool smooth_corners)
{
  std::vector<Eigen::Vector3d> global_path3d;
  global_path3d.reserve(global_path.size());
  global_path3d.emplace_back(global_path.front().x(),
                             global_path.front().y(),
                             0.0);

  double total_length{0.0};
  for(std::size_t i = 1; i < global_path.size(); ++i)
  {
    global_path3d.emplace_back(global_path[i].x(), global_path[i].y(), 0.0);
    total_length += (global_path[i] - global_path[i - 1]).norm();
  }

  if(total_length < min_length)
  {
    std::size_t count = (min_length - total_length) / resample_spacing;
    global_path3d.reserve(global_path3d.size() + count);
    auto const& back_pose = global_path3d.back();
    for(std::size_t i = 0; i < count; ++i)
    {
      auto sample = dir * i * resample_spacing;
      global_path3d.emplace_back(back_pose.x() + sample.x(),
                                 back_pose.y() + sample.y(),
                                 0.0);
    }
  }
  return create_new_line(global_path3d,
                         resample_spacing,
                         min_length,
                         smooth_corners);
}

bool
ReferenceLine::create_new_line(
    algorithms::GridMap<std::uint8_t> const& grid_map,
    std::vector<Eigen::Vector2d> const& global_path,
    Eigen::Vector2d const& pose,
    Eigen::Vector2d const& dir,
    double resample_spacing,
    double min_length,
    bool smooth_corners)
{
  return true;
}

bool
ReferenceLine::create_new_line(std::vector<Eigen::Vector3d> const& global_path,
                               double resample_spacing,
                               double max_length,
                               bool smooth_corners)
{
  valided_ = false;
  truncated_ = false;
  if(global_path.size() < 2 || !std::isfinite(resample_spacing) ||
     !std::isfinite(max_length) || resample_spacing <= kEpsilon)
  {
    REFERENCE_DEBUG_LOG("build reference failed! check param failed!");
    return false;
  }

  log_xy_global_path("/xy/global_path", global_path);

  std::vector<Eigen::Vector2d> source_path;
  if(!extract_distinct_points(global_path, source_path))
  {
    REFERENCE_DEBUG_LOG(
        "build reference failed! extract distinct points failed!");
    return false;
  }

  // 平滑可选，目前先不考虑
  // TODO: 截断、平滑、重采样
  // 最终得到 final_path
  smoothed_path_ = smooth_path(source_path);
  log_xy_path("/xy/smoothed_global_path", smoothed_path_);

  if(!cumulative_lengths())
  {
    REFERENCE_DEBUG_LOG("build reference failed! cumulative lengths failed!");
    return false;
  }

  if(!build_reference_points())
  {
    REFERENCE_DEBUG_LOG(
        "build reference failed! build reference points failed!");
    return false;
  }

  build_lateral_boundary();

  if(!debug())
  {
    // 调试失败通常不阻止主流程
    REFERENCE_DEBUG_LOG("reference line built, but debug generation failed");
  }
  valided_ = true;
  return true;
}

bool
ReferenceLine::extract_distinct_points(
    std::vector<Eigen::Vector3d> const& origin_path,
    std::vector<Eigen::Vector2d>& source_path)
{
  source_path.reserve(origin_path.size());
  for(auto const& pose : origin_path)
  {
    Eigen::Vector2d point;
    point.x() = pose.x();
    point.y() = pose.y();
    if(!std::isfinite(point.x()) || !std::isfinite(point.y()))
    {
      REFERENCE_DEBUG_LOG("Point coordinates are invalid: x:",
                          point.x(),
                          ", y:",
                          point.y());
      return false;
    }
    if(source_path.empty() ||
       std::hypot(point.x() - source_path.back().x(),
                  point.y() - source_path.back().y()) > kEpsilon)
    {
      source_path.push_back(point);
    }
  }

  if(source_path.size() < 2)
  {
    REFERENCE_DEBUG_LOG("The original path consists of only a single point.");
    return false;
  }

  return true;
}

bool
ReferenceLine::cumulative_lengths()
{
  accumulated_s_.clear();
  accumulated_s_.reserve(smoothed_path_.size());
  accumulated_s_.emplace_back(0.0);

  for(int i = 1; i < smoothed_path_.size(); ++i)
  {
    accumulated_s_.emplace_back(
        accumulated_s_.back() +
        std::hypot(smoothed_path_[i].x() - smoothed_path_[i - 1].x(),
                   smoothed_path_[i].y() - smoothed_path_[i - 1].y()));
    if(!truncated_ &&
       accumulated_s_.back() > config_->max_reference_line_length)
    {
      truncated_index_ = i;
      truncated_ = true;
    }
  }
  length_ = accumulated_s_.back();
  if(!truncated_)
  {
    truncated_index_ = smoothed_path_.size();
  }
  length_ = accumulated_s_[truncated_index_ - 1];
  return length_ > kEpsilon;
}

bool
ReferenceLine::build_reference_points(std::size_t start_idx)
{
  if(smoothed_path_.size() < 2 ||
     smoothed_path_.size() != accumulated_s_.size())
  {
    REFERENCE_DEBUG_LOG(
        "build reference failed! path size not equal accumulated_s!");
    return false;
  }
  // 截断后的路径至少需要两个点，避免端点差分访问越界。
  if(truncated_index_ > smoothed_path_.size() ||
     start_idx >= truncated_index_ || truncated_index_ - start_idx < 2)
  {
    return false;
  }
  std::size_t max_idx = truncated_index_ - start_idx;
  reference_points_.clear();
  reference_points_.reserve(max_idx);
  std::vector<double> theta(max_idx, 0.0);
  std::vector<double> tangent_x(max_idx, 0.0);
  std::vector<double> tangent_y(max_idx, 0.0);

  for(std::size_t i = 0; i < max_idx; ++i)
  {
    double dx = NAN;
    double dy = NAN;
    std::size_t path_idx = i + start_idx;
    if(i == 0)
    {
      dx = smoothed_path_[path_idx + 1].x() - smoothed_path_[path_idx].x();
      dy = smoothed_path_[path_idx + 1].y() - smoothed_path_[path_idx].y();
      theta[i] = std::atan2(dy, dx);
      double ds = accumulated_s_[path_idx + 1] - accumulated_s_[path_idx];
      tangent_x[i] = dx / ds;
      tangent_y[i] = dy / ds;
      continue;
    }

    if(i + 1 == max_idx)
    {
      dx = smoothed_path_[path_idx].x() - smoothed_path_[path_idx - 1].x();
      dy = smoothed_path_[path_idx].y() - smoothed_path_[path_idx - 1].y();
      theta[i] = std::atan2(dy, dx);
      double ds = accumulated_s_[path_idx] - accumulated_s_[path_idx - 1];
      tangent_x[i] = dx / ds;
      tangent_y[i] = dy / ds;
      continue;
    }

    // 中间点使用前后两点形成的中心差分方向
    tangent_x[i] =
        (smoothed_path_[path_idx + 1].x() - smoothed_path_[path_idx - 1].x()) /
        (accumulated_s_[path_idx + 1] - accumulated_s_[path_idx - 1]);

    tangent_y[i] =
        (smoothed_path_[path_idx + 1].y() - smoothed_path_[path_idx - 1].y()) /
        (accumulated_s_[path_idx + 1] - accumulated_s_[path_idx - 1]);
    theta[i] = std::atan2(tangent_y[path_idx - start_idx],
                          tangent_x[path_idx - start_idx]);
  }

  std::vector<double> kappa(max_idx, 0.0);

  for(std::size_t i = 1; i + 1 < max_idx; ++i)
  {
    std::size_t path_idx = i + start_idx;
    double const ds =
        accumulated_s_[path_idx + 1] - accumulated_s_[path_idx - 1];

    if(ds > kEpsilon)
    {
      double const dtheta = normalize_angle(theta[i + 1] - theta[i - 1]);

      kappa[i] = dtheta / ds;
    }
  }

  kappa.front() = kappa[1];
  kappa.back() = kappa[max_idx - 2];

  std::vector<double> dkappa(max_idx, 0.0);
  for(std::size_t i = 1; i + 1 < max_idx; ++i)
  {
    std::size_t path_idx = i + start_idx;
    double const ds =
        accumulated_s_[path_idx + 1] - accumulated_s_[path_idx - 1];

    if(ds > kEpsilon)
    {
      dkappa[i] = (kappa[i + 1] - kappa[i - 1]) / ds;
    }
  }

  dkappa.front() = dkappa[1];
  dkappa.back() = dkappa[max_idx - 2];

  reference_points_.clear();
  reference_points_.reserve(max_idx);
  for(std::size_t i = 0; i < max_idx; ++i)
  {
    std::size_t path_idx = i + start_idx;
    reference_points_.emplace_back(
        Vec2d{smoothed_path_[path_idx].x(), smoothed_path_[path_idx].y()},
        theta[i],
        SLPoint{accumulated_s_[path_idx], 0.0},
        kappa[i],
        dkappa[i],
        tangent_x[i],
        tangent_y[i]);
  }

  num_points_ = static_cast<int>(reference_points_.size());
  // 更新截断位置后同步有效末端的 s，保持 length() 和查询范围一致。
  length_ = reference_points_.back().sl().s;
  return true;
}

void
ReferenceLine::build_lateral_boundary()
{
  // 转心限制，相邻两个节点的航向差是 turn
  // 它们的法线会在某个带符号横向距离处相交
  // 这个距离就是 intersection_l = forward * ds / turn
  // ds / |turn| 就是曲率半径 R

  std::vector<SLBoundPoint> cur_boundary(
      reference_points_.size(),
      SLBoundPoint{0.0, config_->upper_bound, config_->lower_bound});
  double kappa_coff{0.7};
  for(std::size_t i = 0; i < reference_points_.size(); ++i)
  {
    auto const& cur_point = reference_points_[i];
    double kappa = cur_point.kappa();
    auto& cur_bound = cur_boundary[i];
    cur_bound.s = cur_point.sl().s;
    if(kappa > 0.1)
    {
      cur_bound.l_upper = std::min(cur_bound.l_upper, kappa_coff / kappa);
    }
    else if(kappa < -0.1)
    {
      // cur_bound.l_lower = std::max(cur_bound.l_lower, -kappa_coff / kappa);
      cur_bound.l_lower = std::max(cur_bound.l_lower, kappa_coff / kappa);
    }
  }

  // 前向：限制离开窄区域后的宽度恢复速度。
  for(std::size_t i = 1; i < reference_points_.size(); ++i)
  {
    auto& cur_bound = cur_boundary[i];
    auto& pre_bound = cur_boundary[i - 1];
    double const ds = cur_bound.s - pre_bound.s;
    double const max_change = config_->rate * ds;
    cur_bound.l_upper =
        std::min(cur_bound.l_upper, pre_bound.l_upper + max_change);
    cur_bound.l_lower =
        std::max(cur_bound.l_lower, pre_bound.l_lower - max_change);
  }

  // 反向：让进入窄区域前的宽度提前收窄。
  for(std::size_t i = reference_points_.size(); i-- > 1;)
  {
    auto const& cur_bound = cur_boundary[i];
    auto& pre_bound = cur_boundary[i - 1];

    double const ds = cur_bound.s - pre_bound.s;
    double const max_change = config_->rate * ds;

    pre_bound.l_upper =
        std::min(pre_bound.l_upper, cur_bound.l_upper + max_change);

    pre_bound.l_lower =
        std::max(pre_bound.l_lower, cur_bound.l_lower - max_change);
  }
  geometry_bound_.set_boundary(std::move(cur_boundary));
}


bool
ReferenceLine::debug()
{
  center_line_.clear();
  left_boundary_sl_.clear();
  right_boundary_sl_.clear();
  auto const& boundary = geometry_bound_.get_boundary();
  std::vector<SLPoint> left_bound;
  std::vector<SLPoint> right_bound;
  left_bound.reserve(reference_points_.size());
  right_bound.reserve(reference_points_.size());
  for(std::size_t i = 0; i < reference_points_.size(); ++i)
  {
    SLPoint temp_sl{reference_points_[i].sl().s, 0.0};
    Vec2d temp_xy{};
    sl_to_xy(temp_sl, &temp_xy);
    center_line_.emplace_back(temp_xy.x, temp_xy.y, 0.0);
    temp_sl.l = boundary[i].l_upper;
    left_bound.emplace_back(temp_sl.s, boundary[i].l_upper);
    right_bound.emplace_back(temp_sl.s, boundary[i].l_lower);
    sl_to_xy(temp_sl, &temp_xy);
    left_boundary_sl_.emplace_back(Eigen::Vector3d(temp_xy.x, temp_xy.y, 0.0));
    temp_sl.l = boundary[i].l_lower;
    sl_to_xy(temp_sl, &temp_xy);
    right_boundary_sl_.emplace_back(Eigen::Vector3d(temp_xy.x, temp_xy.y, 0.0));
  }

  REFERENCE_DEBUG_LOG("reference line debug: center line:",
                      center_line_.size(),
                      ", left_boundary_sl:",
                      left_boundary_sl_.size(),
                      ", right_boundary_sl:",
                      right_boundary_sl_.size());

  log_xy_lane_boundary("/xy/left_boundary", left_boundary_sl_);
  log_xy_lane_boundary("/xy/right_boundary", right_boundary_sl_);
  log_xy_center_line("/xy/center_lane", center_line_);
  log_sl_left_boundary(left_bound);
  log_sl_right_boundary(right_bound);

  return true;
}


std::vector<Eigen::Vector2d>
ReferenceLine::smoothing_gradient(
    std::vector<Eigen::Vector2d> const& points,
    std::vector<Eigen::Vector2d> const& origin_points)
{
  std::size_t const n = points.size();
  std::vector<Eigen::Vector2d> gradient(n);

  for(std::size_t i = 0; i < n; ++i)
  {
    gradient[i] = 2.0 * config_->w_ref * (points[i] - origin_points[i]);
  }

  // 一个二阶差分项同时影响三个点，模板为 [+1, -2, +1]。
  for(std::size_t i = 1; i + 1 < n; ++i)
  {
    Eigen::Vector2d const second_difference =
        points[i - 1] - 2.0 * points[i] + points[i + 1];
    Eigen::Vector2d const term = 2.0 * config_->w_smooth * second_difference;

    gradient[i - 1] += term;
    gradient[i] -= 2.0 * term;
    gradient[i + 1] += term;
  }

  return gradient;
}

Eigen::Vector2d
ReferenceLine::project_into_region(Eigen::Vector2d offset, double radius) const
{
  return {std::clamp(offset.x(), -radius, radius),
          std::clamp(offset.y(), -radius, radius)};
}

std::vector<Eigen::Vector2d>
ReferenceLine::smooth_path(std::vector<Eigen::Vector2d> const& origin_points)
{
  // J 的 Hessian 是 2*w_ref*I + 2*w_smooth*D^T D，D 为二阶差分算子。
  // ||D^T D|| <= ||D||_1 * ||D||_inf = 4 * 4 = 16，故 L <= 2*w_ref + 32*w_smooth。
  // 步长取 1/L 是 J 单调不增的充分条件。
  double const lipschitz = 2.0 * config_->w_ref + 32.0 * config_->w_smooth;
  if(!std::isfinite(lipschitz) || lipschitz <= 0.0)
  {
    REFERENCE_ERROR_LOG("not smoothed");
    return origin_points;
  }
  double const step = 1.0 / lipschitz;
  double max_kappa = get_max_curvature(origin_points);
  double radius = config_->radius;
  if(max_kappa > 1.0)
  {
    radius *= max_kappa;
    radius = std::min(radius, 0.6);
    REFERENCE_DEBUG_LOG("cur kappa is :", max_kappa, ", radius: ", radius);
  }

  std::size_t const n = origin_points.size();
  std::vector<Eigen::Vector2d> current = origin_points;
  std::vector<Eigen::Vector2d> next(n);
  for(std::size_t iter = 0; iter < config_->max_iter; ++iter)
  {
    std::vector<Eigen::Vector2d> const gradient =
        smoothing_gradient(current, origin_points);

    double max_step = 0.0;
    for(std::size_t i = 0; i < n; ++i)
    {
      if(config_->fix_end && (i == 0 || i + 1 == n))
      {
        next[i] = origin_points[i];
      }
      else
      {
        // 梯度下降后的候选点，换算成相对原始点的偏移再做投影。
        Eigen::Vector2d const offset =
            current[i] - step * gradient[i] - origin_points[i];
        next[i] = origin_points[i] + project_into_region(offset, radius);
      }

      max_step = std::max(max_step, (next[i] - current[i]).norm());
    }

    // 所有点计算完毕后统一切换到新一轮。
    current.swap(next);

    if(max_step < config_->tolerance || iter + 1 == config_->max_iter)
    {
      break;
    }
  }
  return current;
}



ReferenceLine::ReferenceSample
ReferenceLine::sample_reference(std::size_t segment, double ratio) const
{
  auto const& start = reference_points_[segment];
  auto const& end = reference_points_[segment + 1U];
  double const span = end.sl().s - start.sl().s;

  auto const horizontal = evaluate_hermite(start.xy().x,
                                           end.xy().x,
                                           start.tangent_x(),
                                           end.tangent_x(),
                                           span,
                                           ratio);
  auto const vertical = evaluate_hermite(start.xy().y,
                                         end.xy().y,
                                         start.tangent_y(),
                                         end.tangent_y(),
                                         span,
                                         ratio);
  return {horizontal.value,
          vertical.value,
          horizontal.first_derivative,
          vertical.first_derivative,
          horizontal.second_derivative,
          vertical.second_derivative};
}

ReferenceLine::Evaluation
ReferenceLine::evaluate_hermite(double first,
                                double last,
                                double first_d,
                                double last_d,
                                double span,
                                double ratio)
{
  double const linear = span * first_d;
  double const quadratic =
      3.0 * (last - first) - span * (2.0 * first_d + last_d);
  double const cubic = 2.0 * (first - last) + span * (first_d + last_d);

  return {first + ratio * (linear + ratio * (quadratic + ratio * cubic)),
          linear + ratio * (2.0 * quadratic + 3.0 * ratio * cubic),
          2.0 * quadratic + 6.0 * ratio * cubic};
}

double
ReferenceLine::get_max_curvature(
    std::vector<Eigen::Vector2d> const& origin_points)
{
  // 不足三个点，约定返回 0。
  if(origin_points.size() < 3)
  {
    return 0.0;
  }

  constexpr double distance_eps = 1e-9;
  double max_curvature = 0.0;

  for(std::size_t i = 1; i + 1 < origin_points.size(); ++i)
  {
    Eigen::Vector2d const a = origin_points[i] - origin_points[i - 1];
    Eigen::Vector2d const b = origin_points[i + 1] - origin_points[i];
    Eigen::Vector2d const c = origin_points[i + 1] - origin_points[i - 1];

    double const len_a = a.norm();
    double const len_b = b.norm();
    double const len_c = c.norm();

    if(len_a <= distance_eps || len_b <= distance_eps || len_c <= distance_eps)
    {
      continue;
    }

    // 二维叉积。
    double const cross = a.x() * b.y() - a.y() * b.x();

    // 接近共线且方向相反：折返不能被误判为直线。
    if(a.dot(b) < 0.0 && std::abs(cross) <= 1e-12 * len_a * len_b)
    {
      continue;
    }

    double const curvature = 2.0 * std::abs(cross) / (len_a * len_b * len_c);

    max_curvature = std::max(max_curvature, curvature);
  }

  return max_curvature;
}


//////////////////////////
/// @brief  机器人左右横向边界查询（lateral boundary query）
///
/// @param grid_map
/// @param global_path
/// @param pose
/// @param dir
//////////////////////////
void
ReferenceLine::build_lateral_boundary(
    algorithms::GridMap<std::uint8_t> const& grid_map,
    Eigen::Vector2d const& pose,
    Eigen::Vector2d const& dir)
{}

} // namespace reference_line
