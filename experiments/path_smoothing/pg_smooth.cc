#include "file.hh"
#include "rerun/debug.hh"
#include "rerun/recording.hh"
#include "path_smoothing.hh"
#include <iostream>
#include <chrono>
#include <stdexcept>
#include <string>
#include <string_view>

int
main(int argc, char** argv)
{
  try
  {
    std::string csv_path = PG_SMOOTH_DEFAULT_PATH;
    bool show_viewer = true;
    algorithms::viz::RecordingOptions recording_options;
    bool verbose = true;
    for(int i = 1; i < argc; ++i)
    {
      std::string_view const argument(argv[i]);
      if(argument == "--help")
      {
        std::cout
            << "Usage: pg_smooth [--path CSV] [--no-viewer] [--save FILE.rrd] "
               "[--quiet] "
               "[--help]\n"
            << "  --path CSV   Load a path from CSV (first two columns: x,y).\n"
            << "               Default: " << PG_SMOOTH_DEFAULT_PATH << '\n'
            << "  --no-viewer  Solve without opening the Rerun viewer.\n"
            << "  --save FILE  Record to a file instead of opening a viewer.\n"
            << "  --quiet      Disable solver iteration logs; still print "
               "Result.\n"
            << "  --help       Show this help and exit.\n"
            << "Exit codes: 0 = converged, 1 = error, 2 = iteration limit "
               "reached.\n";
        return 0;
      }
      if(argument == "--no-viewer")
      {
        show_viewer = false;
      }
      else if(argument == "--quiet")
      {
        verbose = false;
      }
      else if(argument == "--path" || argument == "--save")
      {
        if(i + 1 == argc || std::string_view(argv[i + 1]).starts_with("--"))
        {
          throw std::invalid_argument(std::string(argument) +
                                      " requires a file path");
        }
        if(argument == "--path")
        {
          csv_path = argv[++i];
        }
        else
        {
          recording_options.output = argv[++i];
        }
      }
      else
      {
        throw std::invalid_argument(
            "unknown argument: " + std::string(argument) + " (use --help)");
      }
    }

    recording_options.viewer = show_viewer;
    algorithms::viz::Recording const recording("pg_smooth", recording_options);
    algorithms::Debug::ScopedContext const context(
        {0,
         std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
             .count()});
    auto path = algorithms::load_path_from_csv(csv_path);
    path_smoothing::ProblemOptions po;
    po.fix_endpoints = true;
    po.reference_weight = 20;
    po.smoothness_weight = 100;
    po.tolerance = 1e-8;
    path_smoothing::ProjectedGradientOptions pgo;
    pgo.max_displacement = 1.0;
    pgo.max_iterator = 1000;
    pgo.shape = path_smoothing::RegionShape::Circle;
    pgo.verbose = verbose;
    path_smoothing::Options options;
    options.problem = po;
    options.solver = pgo;

    auto result = path_smoothing::smooth(path, options);
    std::cout << result << '\n';
    algorithms::Debug::flush();
    return result.converged ? 0 : 2;
  }
  catch(std::exception const& error)
  {
    std::cerr << "pg_smooth: " << error.what() << '\n';
    return 1;
  }
}
