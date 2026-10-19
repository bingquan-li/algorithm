#include "scenario.hh"
#include "footprint_io.hh"
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
  auto body =
      timed_call("load_robot_geometry",
                 [&] { return load_robot_geometry(footprint_config); });
  auto const start_pose = timed_call(
      "estimate_footprint_pose",
      [&]
      {
        return estimate_footprint_pose(body, load_footprint(input));
      });

  return {std::move(grid), std::move(body), start_pose};
}

} // namespace narrow_passage
