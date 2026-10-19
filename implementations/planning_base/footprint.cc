#include "footprint.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace algorithms
{
namespace
{
using Point = Eigen::Vector2d;

double
cross(Point const& a, Point const& b)
{
  return a.x() * b.y() - a.y() * b.x();
}

bool
on_segment(Point const& p, Point const& a, Point const& b, double eps)
{
  return std::abs(cross(b - a, p - a)) <= eps &&
         p.x() >= std::min(a.x(), b.x()) - eps &&
         p.x() <= std::max(a.x(), b.x()) + eps &&
         p.y() >= std::min(a.y(), b.y()) - eps &&
         p.y() <= std::max(a.y(), b.y()) + eps;
}

bool
intersects(
    Point const& a, Point const& b, Point const& c, Point const& d, double eps)
{
  double const c1 = cross(b - a, c - a);
  double const c2 = cross(b - a, d - a);
  double const c3 = cross(d - c, a - c);
  double const c4 = cross(d - c, b - c);

  auto const opposite = [eps](double x, double y)
  { return (x > eps && y < -eps) || (x < -eps && y > eps); };

  return (opposite(c1, c2) && opposite(c3, c4)) || on_segment(c, a, b, eps) ||
         on_segment(d, a, b, eps) || on_segment(a, c, d, eps) ||
         on_segment(b, c, d, eps);
}

void
check_simple(std::vector<Point> const& vertices, double eps)
{
  auto const n = vertices.size();
  for(std::size_t i = 0; i < n; ++i)
  {
    std::size_t const next = (i + 1) % n;
    if((vertices[next] - vertices[i]).norm() <= eps)
    {
      throw std::invalid_argument("Footprint has a zero-length edge");
    }

    Point const incoming = vertices[i] - vertices[(i + n - 1) % n];
    Point const outgoing = vertices[next] - vertices[i];
    if(std::abs(cross(incoming, outgoing)) <= eps &&
       incoming.dot(outgoing) < 0.0)
    {
      throw std::invalid_argument("Footprint has overlapping adjacent edges");
    }

    for(std::size_t j = i + 1; j < n; ++j)
    {
      if(next == j || (j + 1) % n == i)
      {
        continue;
      }
      if(intersects(vertices[i],
                    vertices[next],
                    vertices[j],
                    vertices[(j + 1) % n],
                    eps))
      {
        throw std::invalid_argument("Footprint is self-intersecting");
      }
    }
  }
}

} // namespace

Footprint::Footprint(std::string frame_id, Polygon2d vertices)
  : frame_id_(std::move(frame_id))
  , vertices_(std::move(vertices))
{
  if(frame_id_.empty())
  {
    throw std::invalid_argument("Empty footprint frame");
  }
  for(auto const& p : vertices_)
  {
    if(!p.allFinite())
    {
      throw std::invalid_argument("Non-finite footprint vertex");
    }
  }
  if(vertices_.size() > 1 && vertices_.front() == vertices_.back())
  {
    vertices_.pop_back();
  }
  if(vertices_.size() < 3)
  {
    throw std::invalid_argument("Too few footprint vertices");
  }
  auto const bounds = this->bounds();
  double const scale =
      std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
  if(!std::isfinite(scale) || scale <= 0.0)
  {
    throw std::invalid_argument("Degenerate footprint");
  }
  Polygon2d normalized;
  for(auto const& p : vertices_)
  {
    normalized.emplace_back((p - vertices_.front()) / scale);
  }
  check_simple(normalized, 1e-10);
  double area = 0.0;
  for(std::size_t i = 0; i < normalized.size(); ++i)
  {
    area += cross(normalized[i], normalized[(i + 1) % normalized.size()]);
  }
  if(std::abs(area) <= 1e-10)
  {
    throw std::invalid_argument("Degenerate footprint area");
  }
  if(area < 0.0)
  {
    std::reverse(vertices_.begin() + 1, vertices_.end());
  }
}

} // namespace algorithms
