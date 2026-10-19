#include "footprint_io.hh"

#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace narrow_passage
{
namespace
{
using Point = algorithms::Point2d;
using algorithms::BodyExtents;
using algorithms::Footprint;
using algorithms::Polygon2d;
using algorithms::RearAxlePose;
using algorithms::RobotGeometry;

double
cross(Point const& a, Point const& b)
{
  return a.x() * b.y() - a.y() * b.x();
}
} // namespace

Footprint
load_footprint(std::filesystem::path const& path)
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
  if(!data.is_array() || data.size() < 3)
  {
    throw std::invalid_argument(
        "Footprint must contain at least three vertices");
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

  return {"world", std::move(points)};
}


RobotGeometry
load_robot_geometry(std::filesystem::path const& path)
{
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
  auto const& extents = document.at("extents_from_drive_axle");
  auto read_extent = [&](char const* name)
  {
    auto const& value = extents.at(name);
    if(!value.is_number())
    {
      throw std::invalid_argument("Extent must be numeric");
    }
    double const distance = value.get<double>();
    if(!std::isfinite(distance) || distance < 0.0)
    {
      throw std::invalid_argument(
          std::string("Invalid or inconsistent extent: ") + name);
    }
    return distance;
  };
  BodyExtents const dimensions{read_extent("front_extent"),
                               read_extent("rear_extent"),
                               read_extent("left_extent"),
                               read_extent("right_extent")};
  std::optional<Polygon2d> outline;
  if(document.contains("data"))
  {
    Point const axle = read_point(document.at("drive_axle_center"));
    auto const& data = document.at("data");
    if(!data.is_array())
    {
      throw std::invalid_argument("Footprint data must be an array");
    }
    outline.emplace();
    for(auto const& value : data)
    {
      outline->emplace_back(read_point(value) - axle);
    }
  }
  return RobotGeometry(dimensions, std::move(outline));
}

RearAxlePose
estimate_footprint_pose(RobotGeometry const& model,
                        Footprint const& observed,
                        double max_vertex_error)
{
  if(!model.local_footprint())
  {
    throw std::invalid_argument("Pose estimation requires a local outline");
  }
  auto const& body = model.local_footprint()->vertices();
  auto const& world = observed.vertices();
  if(world.size() != body.size() || !std::isfinite(max_vertex_error) ||
     max_vertex_error < 0.0)
  {
    throw std::invalid_argument(
        "Invalid footprint correspondence or tolerance");
  }
  Point p_mean = Point::Zero();
  Point q_mean = Point::Zero();
  for(std::size_t i = 0; i < world.size(); ++i)
  {
    if(!body[i].allFinite() || !world[i].allFinite())
    {
      throw std::invalid_argument("Vertices must be finite");
    }
    p_mean += body[i];
    q_mean += world[i];
  }
  p_mean /= static_cast<double>(world.size());
  q_mean /= static_cast<double>(world.size());
  double a = 0.0;
  double b = 0.0;
  for(std::size_t i = 0; i < world.size(); ++i)
  {
    Point const p = body[i] - p_mean;
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
  for(std::size_t i = 0; i < world.size(); ++i)
  {
    Point const projected = rotation * body[i] + translation;
    if(!projected.allFinite() ||
       (projected - world[i]).norm() > max_vertex_error)
    {
      throw std::invalid_argument(
          "World footprint does not match body configuration; check vertex "
          "order and tolerance");
    }
  }
  RearAxlePose const pose{observed.frame_id(),
                          {translation.x(), translation.y(), rotation.angle()}};
  if(!pose.pose.allFinite())
  {
    throw std::invalid_argument("Fitted axle position must be finite");
  }
  return pose;
}

} // namespace narrow_passage
