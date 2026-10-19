#include "geometry.hh"


#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>


namespace algorithms::local_escape_detail
{
Eigen::Vector2d
direction(Pose const& pose)
{
  return {std::cos(pose.yaw), std::sin(pose.yaw)};
}

// 车体坐标中，以 X=x 切多边形，返回横向最小/最大值。
std::pair<double, double>
lateral_extent(std::span<Eigen::Vector2d const> footprint, double x)
{
  double low = std::numeric_limits<double>::infinity();
  double high = -low;
  for(std::size_t i = 0; i < footprint.size(); ++i)
  {
    auto const& a = footprint[i];
    auto const& b = footprint[(i + 1) % footprint.size()];
    if(x < std::min(a.x(), b.x()) || x > std::max(a.x(), b.x()))
    {
      continue;
    }
    if(a.x() == b.x())
    {
      low = std::min({low, a.y(), b.y()});
      high = std::max({high, a.y(), b.y()});
    }
    else
    {
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

// 简单多边形到闭合障碍格的距离；接触、交叉和包含均返回零。
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

// 常曲率精确积分；有符号弧长控制前进/倒车，零弧长退化为原地旋转。
Pose
advance(Pose const& start, Motion const& motion, double fraction)
{
  Pose result = start;
  double const ds = motion.distance * fraction;
  double const angle = motion.rotation * fraction + motion.curvature * ds;
  double const half = 0.5 * angle;
  double const sinc =
      std::abs(half) < 1e-8 ? 1.0 - half * half / 6.0 : std::sin(half) / half;
  result.position +=
      ds * sinc *
      Eigen::Vector2d(std::cos(start.yaw + half), std::sin(start.yaw + half));
  result.yaw += angle;
  return result;
}
} // namespace algorithms::local_escape_detail
