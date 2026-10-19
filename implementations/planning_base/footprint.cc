#include "footprint.hh"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
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

void
check_options(FootprintOptions const& options)
{
  if(!std::isfinite(options.relative_tolerance) ||
     options.relative_tolerance <= 0.0 || options.relative_tolerance >= 0.01 ||
     !std::isfinite(options.rear_axle_offset) ||
     options.rear_axle_offset < 0.0 ||
     !std::isfinite(options.min_axis_anisotropy) ||
     !std::isfinite(options.min_end_width_difference) ||
     options.width_samples == 0)
  {
    throw std::invalid_argument("Invalid FootprintOptions");
  }
}

// projected[i] = (longitudinal, lateral) in normalized coordinates.
double
section_width(std::vector<Point> const& projected, double s)
{
  std::vector<double> hits;
  hits.reserve(projected.size());

  for(std::size_t i = 0; i < projected.size(); ++i)
  {
    Point const& a = projected[i];
    Point const& b = projected[(i + 1) % projected.size()];
    if(std::min(a.x(), b.x()) <= s && s < std::max(a.x(), b.x()))
    {
      hits.push_back(a.y() + (s - a.x()) * (b.y() - a.y()) / (b.x() - a.x()));
    }
  }

  std::sort(hits.begin(), hits.end());
  double width = 0.0;
  // Convex or simple polygon: pair up crossings.
  for(std::size_t i = 1; i < hits.size(); i += 2)
  {
    width += hits[i] - hits[i - 1];
  }
  return width;
}

double
mean_width(std::vector<Point> const& projected,
           double s_min,
           double length,
           double begin,
           double end,
           std::size_t samples)
{
  double sum = 0.0;
  for(std::size_t i = 0; i < samples; ++i)
  {
    double const fraction = begin + (end - begin) *
                                        (static_cast<double>(i) + 0.5) /
                                        static_cast<double>(samples);
    sum += section_width(projected, s_min + length * fraction);
  }
  return sum / static_cast<double>(samples);
}

} // namespace

Footprint
make_footprint(std::vector<Point> vertices, FootprintOptions const& options)
{
  check_options(options);

  for(Point const& p : vertices)
  {
    if(!p.allFinite())
    {
      throw std::invalid_argument("Footprint vertices must be finite");
    }
  }

  if(vertices.size() == 9 && vertices.front() == vertices.back())
  {
    vertices.pop_back();
  }
  if(vertices.size() != 8)
  {
    throw std::invalid_argument("Footprint must contain eight vertices");
  }

  // Normalize world coordinates to avoid cancellation in area moments.
  Point const origin = vertices.front();
  double scale = 0.0;
  for(Point const& p : vertices)
  {
    scale = std::max(scale, (p - origin).cwiseAbs().maxCoeff());
  }
  if(!std::isfinite(scale) || scale <= 0.0)
  {
    throw std::invalid_argument("Invalid footprint coordinate range");
  }

  std::vector<Point> local;
  local.reserve(vertices.size());
  for(Point const& p : vertices)
  {
    local.emplace_back((p - origin) / scale);
  }
  // Reject invalid topology, but do not require exact left-right symmetry.
  check_simple(local, options.relative_tolerance);

  // Green's theorem: centroid and covariance of the filled polygon.
  long double a2 = 0.0L;
  long double mx = 0.0L;
  long double my = 0.0L;
  long double mxx = 0.0L;
  long double myy = 0.0L;
  long double mxy = 0.0L;

  for(std::size_t i = 0; i < local.size(); ++i)
  {
    Point const& a = local[i];
    Point const& b = local[(i + 1) % local.size()];
    long double const x = a.x();
    long double const y = a.y();
    long double const u = b.x();
    long double const v = b.y();
    long double const c = x * v - u * y;
    a2 += c;
    mx += (x + u) * c;
    my += (y + v) * c;
    mxx += (x * x + x * u + u * u) * c;
    myy += (y * y + y * v + v * v) * c;
    mxy += (2 * x * y + x * v + u * y + 2 * u * v) * c;
  }

  if(std::abs(a2) <= options.relative_tolerance)
  {
    throw std::invalid_argument("Footprint area is zero or too small");
  }

  long double const cx = mx / (3.0L * a2);
  long double const cy = my / (3.0L * a2);
  Point const area_center(static_cast<double>(cx), static_cast<double>(cy));

  auto const xx = static_cast<double>(mxx / (6.0L * a2) - cx * cx);
  auto const yy = static_cast<double>(myy / (6.0L * a2) - cy * cy);
  auto const xy = static_cast<double>(mxy / (12.0L * a2) - cx * cy);
  double const trace = xx + yy;
  if(!std::isfinite(trace) || trace <= 0.0)
  {
    throw std::invalid_argument("Footprint covariance is invalid");
  }

  // Assume the longitudinal distribution is the major principal axis.
  // For a nearly isotropic polygon, this is just an approximate direction.
  double const theta = 0.5 * std::atan2(2.0 * xy, xx - yy);
  Point forward(std::cos(theta), std::sin(theta));
  Point const left(-forward.y(), forward.x());

  std::vector<Point> projected;
  projected.reserve(local.size());
  double s_min = std::numeric_limits<double>::infinity();
  double s_max = -std::numeric_limits<double>::infinity();
  for(Point const& p : local)
  {
    Point const delta = p - area_center;
    double const s = delta.dot(forward);
    projected.emplace_back(s, delta.dot(left));
    s_min = std::min(s_min, s);
    s_max = std::max(s_max, s);
  }

  // Front is narrower; compare average cross-section widths.
  double const rear_width = mean_width(projected,
                                       s_min,
                                       s_max - s_min,
                                       0.2,
                                       0.4,
                                       options.width_samples);
  double const front_width = mean_width(projected,
                                        s_min,
                                        s_max - s_min,
                                        0.6,
                                        0.8,
                                        options.width_samples);

  if(front_width > rear_width)
  {
    forward = -forward;
    for(Point& p : projected)
    {
      p = -p; // Both the longitudinal and left axes reverse.
    }
  }
  // Close widths no longer cause an exception. The sign of the smaller
  // difference is used, so the heading may be ambiguous in that case.

  double final_s_min = std::numeric_limits<double>::infinity();
  double final_s_max = -std::numeric_limits<double>::infinity();
  double d_min = std::numeric_limits<double>::infinity();
  double d_max = -std::numeric_limits<double>::infinity();
  for(Point const& p : projected)
  {
    final_s_min = std::min(final_s_min, p.x());
    final_s_max = std::max(final_s_max, p.x());
    d_min = std::min(d_min, p.y());
    d_max = std::max(d_max, p.y());
  }

  double const body_length = (final_s_max - final_s_min) * scale;
  if(options.rear_axle_offset > body_length)
  {
    throw std::invalid_argument("rear_axle_offset exceeds body length");
  }

  // Approximate the symmetry axis by the midpoint of lateral extrema,
  // rather than assuming the area centroid lies on an exact symmetry axis.
  double const lateral_mid = 0.5 * (d_min + d_max);
  double const axle_s = final_s_min + options.rear_axle_offset / scale;
  Point const final_left(-forward.y(), forward.x());
  Point const rear_axle_local =
      area_center + axle_s * forward + lateral_mid * final_left;

  Footprint result;
  result.vertices = std::move(vertices);
  result.centroid = origin + scale * rear_axle_local;
  result.forward = forward;
  result.front_extent = (final_s_max - axle_s) * scale;
  result.rear_extent = (axle_s - final_s_min) * scale;
  result.left_extent = (d_max - lateral_mid) * scale;
  result.right_extent = (lateral_mid - d_min) * scale;
  return result;
}

std::vector<Eigen::Vector2d>
closed_vertices(Footprint const& footprint)
{
  auto result = footprint.vertices;
  if(!result.empty())
  {
    result.push_back(result.front());
  }
  return result;
}

std::vector<Eigen::Vector2d>
load_footprint_vertices(std::filesystem::path const& path)
{
  std::ifstream input(path);
  if(!input)
  {
    throw std::runtime_error("Cannot open footprint file: " + path.string());
  }
  auto const document = nlohmann::json::parse(input);
  auto const& footprint = document.at("foot_print");
  if(footprint.value("coordinate_system", std::string("world")) != "world" ||
     footprint.value("unit", std::string("m")) != "m")
  {
    throw std::invalid_argument(
        "Footprint must use world coordinates in meters");
  }

  auto const& data = footprint.at("data");
  if(!data.is_array() || (data.size() != 8 && data.size() != 9))
  {
    throw std::invalid_argument("Footprint must contain eight vertices");
  }

  std::vector<Point> points;
  points.reserve(data.size());
  for(auto const& vertex : data)
  {
    if(!vertex.is_array() || vertex.size() != 2 || !vertex[0].is_number() ||
       !vertex[1].is_number())
    {
      throw std::invalid_argument("Footprint vertex must be [x, y]");
    }
    Point const p(vertex[0].get<double>(), vertex[1].get<double>());
    if(!p.allFinite())
    {
      throw std::invalid_argument("Footprint vertices must be finite");
    }
    points.push_back(p);
  }

  if(points.size() == 9)
  {
    if(points.front() != points.back())
    {
      throw std::invalid_argument("Ninth vertex must repeat the first");
    }
    points.pop_back();
  }
  return points;
}

Footprint
load_footprint(std::filesystem::path const& path,
               FootprintOptions const& options)
{
  // The body configuration is authoritative; options only control validation.
  if(!std::isfinite(options.relative_tolerance) ||
     options.relative_tolerance <= 0.0 || options.relative_tolerance >= 0.01)
  {
    throw std::invalid_argument("Invalid footprint validation tolerance");
  }
  std::ifstream input(path);
  if(!input)
  {
    throw std::runtime_error("Cannot open footprint configuration: " +
                             path.string());
  }
  auto const document = nlohmann::json::parse(input);
  if(document.at("coordinate_system") != "body" || document.at("unit") != "m" ||
     document.at("axes").at("x_positive") != "forward" ||
     document.at("axes").at("y_positive") != "left")
  {
    throw std::invalid_argument(
        "Expected body coordinates in meters, +X forward, +Y left");
  }
  auto read_point = [](nlohmann::json const& value) -> Point
  {
    if(!value.is_array() || value.size() != 2 || !value[0].is_number() ||
       !value[1].is_number())
    {
      throw std::invalid_argument("Expected [x, y]");
    }
    Point const p(value[0].get<double>(), value[1].get<double>());
    if(!p.allFinite())
    {
      throw std::invalid_argument("Coordinates must be finite");
    }
    return p;
  };
  Footprint result;
  result.centroid = read_point(document.at("drive_axle_center"));
  auto const& data = document.at("data");
  if(!data.is_array() || data.size() != 8)
  {
    throw std::invalid_argument(
        "Body configuration requires eight unclosed vertices");
  }
  for(auto const& value : data)
  {
    result.vertices.push_back(read_point(value));
  }
  Point minimum = result.vertices.front() - result.centroid;
  Point maximum = minimum;
  for(auto const& vertex : result.vertices)
  {
    Point const delta = vertex - result.centroid;
    minimum = minimum.cwiseMin(delta);
    maximum = maximum.cwiseMax(delta);
  }
  double const scale = (maximum - minimum).maxCoeff();
  if(!std::isfinite(scale) || scale <= 0.0)
  {
    throw std::invalid_argument("Degenerate body footprint");
  }
  std::vector<Point> normalized;
  normalized.reserve(result.vertices.size());
  for(auto const& vertex : result.vertices)
  {
    normalized.emplace_back((vertex - result.vertices.front()) / scale);
  }
  check_simple(normalized, options.relative_tolerance);
  double area = 0.0;
  for(std::size_t i = 0; i < normalized.size(); ++i)
  {
    area += cross(normalized[i], normalized[(i + 1) % normalized.size()]);
  }
  if(std::abs(area) <= options.relative_tolerance)
  {
    throw std::invalid_argument("Degenerate body footprint area");
  }
  auto const& extents = document.at("extents_from_drive_axle");
  auto read_extent = [&](char const* name, double projected)
  {
    auto const& value = extents.at(name);
    if(!value.is_number())
    {
      throw std::invalid_argument("Extent must be numeric");
    }
    double const distance = value.get<double>();
    if(!std::isfinite(distance) || distance < 0.0 ||
       std::abs(distance - projected) > 1e-6)
    {
      throw std::invalid_argument(
          std::string("Invalid or inconsistent extent: ") + name);
    }
    return distance;
  };
  result.front_extent = read_extent("front_extent", maximum.x());
  result.rear_extent = read_extent("rear_extent", -minimum.x());
  result.left_extent = read_extent("left_extent", maximum.y());
  result.right_extent = read_extent("right_extent", -minimum.y());
  return result;
}

Footprint
place_footprint(Footprint const& body,
                std::vector<Point> const& world,
                double max_vertex_error)
{
  if(body.vertices.size() != 8 || world.size() != body.vertices.size() ||
     !std::isfinite(max_vertex_error) || max_vertex_error < 0.0)
  {
    throw std::invalid_argument(
        "Invalid footprint correspondence or tolerance");
  }
  Point p_mean = Point::Zero();
  Point q_mean = Point::Zero();
  for(std::size_t i = 0; i < world.size(); ++i)
  {
    if(!body.vertices[i].allFinite() || !world[i].allFinite())
    {
      throw std::invalid_argument("Vertices must be finite");
    }
    p_mean += body.vertices[i];
    q_mean += world[i];
  }
  p_mean /= static_cast<double>(world.size());
  q_mean /= static_cast<double>(world.size());
  double a = 0.0;
  double b = 0.0;
  for(std::size_t i = 0; i < world.size(); ++i)
  {
    Point const p = body.vertices[i] - p_mean;
    Point const q = world[i] - q_mean;
    a += p.dot(q);
    b += cross(p, q);
  }
  if(!std::isfinite(a) || !std::isfinite(b) || std::hypot(a, b) <= 1e-15)
  {
    throw std::invalid_argument("Cannot determine footprint rotation");
  }
  Eigen::Rotation2Dd const rotation(std::atan2(b, a));
  Point const translation = q_mean - rotation * p_mean;
  Footprint result = body;
  for(std::size_t i = 0; i < world.size(); ++i)
  {
    result.vertices[i] = rotation * body.vertices[i] + translation;
    if((result.vertices[i] - world[i]).norm() > max_vertex_error)
    {
      throw std::invalid_argument(
          "World footprint does not match body configuration; check vertex "
          "order and tolerance");
    }
  }
  result.centroid = rotation * body.centroid + translation;
  result.forward = rotation * Point::UnitX();
  return result;
}

} // namespace algorithms
