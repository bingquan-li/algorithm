#include "scenario.hh"
#include "timing.hh"
#include "visualization.hh"


#include <iostream>


int
main(int argc, char** argv)
{
  using namespace narrow_passage;
  try
  {
    auto const options = parse_options(argc, argv);
    if(options.help)
    {
      std::cout << "Usage: narrow_passage_reversing [--input frame.json] "
                   "[--blueprint layout.rbl] [--no-viewer]\n";
      return 0;
    }

    ScopedTimer const total_timer("total (includes optional Rerun lifetime)");
    // 这里默认加载的是输出的全局坐标下的 footprint
    auto const scenario = load_scenario(options.input);
    algorithms::Config const config;
    auto const result =
        timed_call("planner.plan",
                   [&]
                   {
                     return algorithms::plan(scenario.grid,
                                             scenario.start,
                                             scenario.body_footprint,
                                             config);
                   });

    // 失败时也显示已生成的候选，便于定位被拒绝的位置。
    if(options.viewer)
    {
      visualize(scenario, result, config.occupied_threshold, options.blueprint);
    }

    if(result.status != algorithms::Status::success || !result.selected)
    {
      std::cerr << "Planning failed: "
                << (result.status == algorithms::Status::start_blocked
                        ? "start_blocked"
                        : "no_path")
                << '\n';
      return 1;
    }
    auto const& selected = result.candidates[*result.selected];
    std::cout << "Planning succeeded: reverse_progress="
              << selected.reverse_progress
              << ", min_clearance=" << selected.min_clearance
              << ", candidates=" << result.candidates.size() << '\n';
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
