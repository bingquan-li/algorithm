#include "straight_prefix.hh"
#include "reference_line.h"
#include "debug.h"
#include "rerun/recording.hh"
#include "rerun/debug.hh"
#include <chrono>

#include "grid_map.hh"

#include <iostream>
#include <string_view>

int
main(int argc, char** argv)
{
  try
  {
    algorithms::viz::RecordingOptions recording_options;
    bool viewer = true;
    for(int i = 1; i < argc; ++i)
    {
      std::string_view const arg(argv[i]);
      if(arg == "--help")
      {
        std::cout
            << "Usage: path_orientation [--no-viewer] [--save FILE.rrd]\n";
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
      else
      {
        throw std::invalid_argument("Unknown argument: " + std::string(arg));
      }
    }
    recording_options.viewer = viewer;
    algorithms::viz::Recording const recording("reference_line",
                                               recording_options);
    algorithms::Debug::ScopedContext const context(
        {0,
         std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
             .count()});
    auto path = load_path(
        "/workspace/algorithms/data/narrow_passage_reverseing2/path.csv");
    auto result = extract(path);

    std::vector<Eigen::Vector2d> direction_path{path.begin(),
                                                path.begin() +
                                                    result.end_index};

    // reference_line::log_xy_path("/origin_path", path);
    // reference_line::log_xy_path("/straight_path", direction_path);


    reference_line::Config config;
    reference_line::ReferenceLine ref(&config);
    if(direction_path.empty())
    {
      std::cout << "err construct direction path failed!";
      return 1;
    }

    Eigen::Vector2d vehicle_dir(0.2383018770177138, 0.9711911322751224);
    // if(std::abs(result.direction.dot(vehicle_dir)) > 0.03)
    // {
    //   result.direction = -vehicle_dir;
    // }

    std::filesystem::path grid_map_path(
        "/workspace/algorithms/data/narrow_passage_reverseing2/frame.json");

    auto grid_map = algorithms::load_grid_map_json(grid_map_path);

    ref.create_new_line(direction_path, result.direction, 0.1, 2.2);
    // ref.create_new_line(path3d);

    algorithms::Debug::flush();
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << "path_orientation: " << error.what() << '\n';
    return 1;
  }
}
