#include "footprint.hh"
#include "rerun/recording.hh"
#include "rerun/debug.hh"

#include <iostream>
#include <nlohmann/json.hpp>
#include <random>
#include <rerun.hpp>

#include <fstream>
#include <numbers>


using Point2d = Eigen::Vector2d;
using Point = Point2d;

using namespace algorithms;

RobotGeometry
loadRobotGeometry(std::filesystem::path const& path)
{
  std::ifstream input(path);
  if(!input)
  {
    throw std::runtime_error("Cannot open footprint configuration: " +
                             path.string());
  }
  auto const document = nlohmann::json::parse(input);
  if(document.at("coordinate_system") != "base_link" ||
     document.at("unit") != "m" ||
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

  BodyExtents const dimensions{
      read_extent("front_extent"),
      read_extent("rear_extent"),
      read_extent("left_extent"),
      read_extent("right_extent"),
  };

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

  std::string frame_id = "map";
  if(document.contains("coordinate_system"))
  {
    frame_id = document.at("coordinate_system");
  }
  return RobotGeometry(dimensions, frame_id, std::move(outline));
}

std::vector<Eigen::Vector2d>
transformed(std::span<Eigen::Vector2d const> outline)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(outline.size());
  for(auto const& p : outline)
  {
    points.emplace_back(p);
  }
  return points;
}

double
cross2d(Eigen::Vector2d const& a, Eigen::Vector2d const& b)
{
  return a.x() * b.y() - a.y() * b.x();
}

double
estimate_footprint_theta(std::vector<Eigen::Vector2d> const& outline1,
                         std::vector<Eigen::Vector2d> const& outline2)
{
  double dot{0.0};
  double cross{0.0};
  for(std::size_t i = 0; i < outline1.size(); ++i)
  {
    dot += outline1[i].dot(outline2[i]);
    cross += cross2d(outline1[i], outline2[i]);
  }
  return std::atan2(cross, dot);
}


int
main()
{
  try
  {
    algorithms::viz::Recording const recording("least_square",
                                               {
                                                   .viewer = true,
                                               });
    auto footprint =
        loadRobotGeometry("/workspace/algorithms/data/footprint.json");
    Debug::ScopedContext const context(
        {.frame_id = 0, .timestamp_ns = 0, .step = 0});
    Debug::line("base_link",
                footprint.local_footprint()->vertices(),
                Debug::Colors::red,
                0.008,
                0.02,
                true);

    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist(-std::numbers::pi,
                                                std::numbers::pi);
    double const theta = dist(gen);
    double const c = std::cos(theta);
    double const s = std::sin(theta);

    Debug::arrow("theta", {0., 0.}, {c, s}, Debug::Colors::orange);
    Eigen::Matrix2d rotation;
    // 这里描述的逆时针旋转矩阵
    rotation << c, -s, s, c;
    std::vector<Eigen::Vector2d> rotated;
    rotated.reserve(footprint.local_footprint()->vertices().size());
    for(auto const& point : footprint.local_footprint()->vertices())
    {
      rotated.emplace_back(rotation * point);
    }
    Debug::line("rotated", rotated, Debug::Colors::blue, 0.008, 0.02, true);
    auto r = estimate_footprint_theta(footprint.local_footprint()->vertices(),
                                      rotated);
    std::cout << "theta: " << theta << ", estimate result: " << r << '\n';
  }
  catch(std::exception const& e)
  {
    std::printf("exception: %s\n", e.what());
  }

  return 0;
}

// 原来是自己添加的日志导致的错误。。。
// polygon 就是 line，不要一直想着是 polygon
