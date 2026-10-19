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
  options.footprint = NARROW_PASSAGE_DEFAULT_FOOTPRINT;
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
    else if(arg == "--input" || arg == "--blueprint" || arg == "--footprint")
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
      else if(arg == "--footprint")
      {
        options.footprint = path;
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
load_scenario(std::filesystem::path const& input,
              std::filesystem::path const& footprint_config)
{
  auto grid = timed_call("load_grid_map_json",
                         [&] { return algorithms::load_grid_map_json(input); });
  auto const configured =
      timed_call("load_footprint_config",
                 [&] { return algorithms::load_footprint(footprint_config); });
  auto footprint = timed_call("fit_footprint_pose",
                              [&]
                              {
                                return algorithms::place_footprint(
                                    configured,
                                    algorithms::load_footprint_vertices(input));
                              });

  algorithms::Pose const start{footprint.centroid, footprint.yaw()};
  std::vector<Eigen::Vector2d> body;
  body.reserve(configured.vertices.size());
  for(auto const& vertex : configured.vertices)
  {
    body.emplace_back(vertex - configured.centroid);
  }
  return {std::move(grid), std::move(footprint), start, std::move(body)};
}

} // namespace narrow_passage
