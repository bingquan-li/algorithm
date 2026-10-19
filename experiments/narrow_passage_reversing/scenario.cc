#include "scenario.hh"
#include "timing.hh"

#include <stdexcept>
#include <string_view>
#include <utility>


namespace narrow_passage
{

Options
parse_options(int argc, char const* const* argv)
{
  Options options;
  options.input = NARROW_PASSAGE_DEFAULT_INPUT;
  for(int i = 1; i < argc; ++i)
  {
    std::string_view const arg(argv[i]);
    if(arg == "--help" || arg == "-h")
    {
      options.help = true;
    }
    else if(arg == "--no-viewer")
    {
      options.viewer = false;
    }
    else if(arg == "--input" || arg == "--blueprint")
    {
      if(i + 1 >= argc)
      {
        throw std::invalid_argument(std::string(arg) + " requires a path");
      }
      auto const path = std::filesystem::path(argv[++i]);
      if(arg == "--input")
      {
        options.input = path;
      }
      else
      {
        options.blueprint = path;
      }
    }
    else
    {
      throw std::invalid_argument("Unknown argument: " + std::string(arg));
    }
  }
  return options;
}


Scenario
load_scenario(std::filesystem::path const& input)
{
  auto grid = timed_call("load_grid_map_json",
                         [&] { return algorithms::load_grid_map_json(input); });
  auto footprint =
      timed_call("load_footprint",
                 [&] { return algorithms::load_footprint(input); });

  algorithms::Pose const start{footprint.centroid, footprint.yaw()};
  std::vector<Eigen::Vector2d> body;
  body.reserve(footprint.vertices.size());
  for(auto const& vertex : footprint.vertices)
  {
    Eigen::Vector2d const delta = vertex - start.position;
    body.emplace_back(delta.dot(footprint.forward),
                      delta.dot(footprint.left()));
  }
  return {std::move(grid), std::move(footprint), start, std::move(body)};
}

} // namespace narrow_passage
