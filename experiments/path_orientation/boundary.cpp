#include "boundary.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace reference_line
{

double
SLBoundary::get_lower_bound_by_s(double s) const
{
  return get_interpolated_s_width(s, false);
}

double
SLBoundary::get_upper_bound_by_s(double s) const
{
  return get_interpolated_s_width(s, true);
}

double
SLBoundary::get_interpolated_s_width(double const s, bool upper) const
{
  if(boundary_.empty())
  {
    return 0;
  }

  if(!std::isfinite(s))
  {
    return 0;
  }

  auto const value = [upper](SLBoundPoint const& point)
  { return upper ? point.l_upper : point.l_lower; };

  // 超出范围时，返回最近端点的边界值。
  // 同时覆盖只有一个点的情况。
  if(s <= boundary_.front().s)
  {
    return value(boundary_.front());
  }

  if(s >= boundary_.back().s)
  {
    return value(boundary_.back());
  }

  auto const right =
      std::lower_bound(boundary_.begin(),
                       boundary_.end(),
                       s,
                       [](SLBoundPoint const& point, double query_s)
                       { return point.s < query_s; });

  // 查询位置恰好是一个采样点。
  if(right->s == s)
  {
    return value(*right);
  }

  auto const left = std::prev(right);

  double const t = (s - left->s) / (right->s - left->s);

  return (1.0 - t) * value(*left) + t * value(*right);
}

SLBoundPoint
SLBoundary::get_bound_point_by_s(double const s) const
{
  if(boundary_.empty())
  {
    return SLBoundPoint{s, 0, 0};
  }

  if(!std::isfinite(s))
  {
    return SLBoundPoint{s, 0, 0};
  }

  if(s <= boundary_.front().s)
  {
    return SLBoundPoint{s,
                        boundary_.front().l_upper,
                        boundary_.front().l_lower};
  }

  if(s >= boundary_.back().s)
  {
    return SLBoundPoint{s, boundary_.back().l_upper, boundary_.back().l_lower};
  }

  auto const right =
      std::lower_bound(boundary_.begin(),
                       boundary_.end(),
                       s,
                       [](SLBoundPoint const& point, double query_s)
                       { return point.s < query_s; });

  if(right->s == s)
  {
    return *right;
  }

  auto const left = std::prev(right);
  double const t = (s - left->s) / (right->s - left->s);

  SLBoundPoint result;
  result.s = s;
  result.l_upper = (1.0 - t) * left->l_upper + t * right->l_upper;
  result.l_lower = (1.0 - t) * left->l_lower + t * right->l_lower;
  return result;
}

SLBoundary
SLBoundary::intersect(double const l_lower, double const l_upper) const
{
  SLBoundary result(*this);

  for(auto& point : result.boundary_)
  {
    point.l_upper = std::min(point.l_upper, l_upper);
    point.l_lower = std::max(point.l_lower, l_lower);
  }

  return result;
}

void
SLBoundary::include(double min_l, double max_l)
{
  if(!std::isfinite(min_l) || !std::isfinite(max_l))
  {
    return;
  }

  for(auto& point : boundary_)
  {
    point.l_upper = std::max(point.l_upper, max_l);
    point.l_lower = std::min(point.l_lower, min_l);
  }
}

double
SLBoundary::min_width() const
{
  if(boundary_.empty())
  {
    return 0.0;
  }

  double width = std::numeric_limits<double>::infinity();
  for(auto const& point : boundary_)
  {
    width = std::min(width, point.l_upper - point.l_lower);
  }

  return width;
}

} // namespace reference_line