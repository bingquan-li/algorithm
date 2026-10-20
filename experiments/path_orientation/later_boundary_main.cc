#include "later_boundary_query.hh"
#include "local_escape_planner.hh"

#include "rerun/debug.hh"
#include "rerun/recording.hh"
#include <nlohmann/json.hpp>

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string_view>

namespace
{
std::vector<Eigen::Vector2d>
load_footprint(std::filesystem::path const& path)
{
  std::ifstream input(path);
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
  std::vector<Eigen::Vector2d> points;
  for(auto const& vertex : data)
  {
    if(!vertex.is_array() || vertex.size() != 2)
    {
      throw std::invalid_argument("Footprint vertices must contain [x, y]");
    }
    Eigen::Vector2d const point(vertex.at(0).get<double>(),
                                vertex.at(1).get<double>());
    if(!point.allFinite())
    {
      throw std::invalid_argument("Footprint vertices must be finite");
    }
    points.push_back(point);
  }
  if(points.front() != points.back())
  {
    points.push_back(points.front());
  }
  return points;
}

} // namespace

int
main(int argc, char** argv)
{
  try
  {
    std::filesystem::path grid_map_path =
        "/workspace/algorithms/data/narrow_passage_reverseing2/frame.json";
    algorithms::viz::RecordingOptions recording_options;
    // Preserve the previously documented environment setting; CLI takes precedence.
    if(auto const* output = std::getenv("LATTICE_RERUN_SAVE");
       (output != nullptr) && *output)
    {
      recording_options.output = output;
    }
    bool viewer = true;
    bool has_input = false;
    for(int i = 1; i < argc; ++i)
    {
      std::string_view const arg(argv[i]);
      if(arg == "--help")
      {
        std::cout << "Usage: later_boundary [frame.json] [--no-viewer] [--save "
                     "FILE.rrd]\n";
        return 0;
      }
      if(arg == "--no-viewer")
      {
        viewer = false;
      }
      else if(arg == "--save")
      {
        if(++i == argc)
        {
          throw std::invalid_argument("--save requires a file path");
        }
        recording_options.output = argv[i];
      }
      else if(!arg.starts_with("--") && !has_input)
      {
        grid_map_path = argv[i];
        has_input = true;
      }
      else
      {
        throw std::invalid_argument("Unknown argument: " + std::string(arg));
      }
    }
    recording_options.viewer = viewer;
    algorithms::viz::Recording const recording("lateral_boundary",
                                               recording_options);
    algorithms::Debug::ScopedContext const context(
        {0,
         std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
             .count()});
    auto const grid_map = algorithms::load_grid_map_json(grid_map_path);
    auto const footprint = load_footprint(grid_map_path);
    Eigen::Vector2d const vehicle_pose(-2.519150772556948,
                                       -0.48583814710976636);
    Eigen::Vector2d const vehicle_dir(0.2383018770177138, 0.9711911322751224);

    // 示例：车后 0.5 m 至车前 0.5 m，每 0.1 m 查询一个中心线点。
    std::vector<double> longitudinal_samples;
    for(int i = -5; i <= 5; ++i)
    {
      longitudinal_samples.push_back(i * 0.1);
    }
    auto const boundary = build_lateral_boundary(grid_map,
                                                 vehicle_pose,
                                                 vehicle_dir,
                                                 longitudinal_samples);
    auto const& [left_width, right_width] = boundary;
    std::cout << "s_m,left_width_m,right_width_m\n";
    for(std::size_t i = 0; i < longitudinal_samples.size(); ++i)
    {
      std::cout << longitudinal_samples[i] << ',' << left_width[i] << ','
                << right_width[i] << '\n';
    }


    Eigen::Vector2d const forward = vehicle_dir.normalized();
    Eigen::Vector2d const left(-forward.y(), forward.x());
    std::vector<Eigen::Vector2d> body_footprint;
    for(auto const& point : footprint)
    {
      Eigen::Vector2d const offset = point - vehicle_pose;
      body_footprint.emplace_back(offset.dot(forward), offset.dot(left));
    }
    if(body_footprint.front() == body_footprint.back())
    {
      body_footprint.pop_back();
    }

    local_escape::Config planner_config;
    // vehicle_pose 应对应差速底盘的旋转中心；配置单位：m、rad、1/m。
    local_escape::Pose const start{vehicle_pose,
                                   std::atan2(forward.y(), forward.x())};
    auto const plan =
        local_escape::plan(grid_map, start, body_footprint, planner_config);

    algorithms::Debug::flush();
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
