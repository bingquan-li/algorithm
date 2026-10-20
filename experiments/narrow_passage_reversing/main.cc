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
                   "[--footprint footprint.json] [--blueprint layout.rbl] "
                   "[--no-viewer]\n";
      return 0;
    }

    ScopedTimer const total_timer("total (includes optional Rerun lifetime)");
    // 配置提供车体几何和轴心，全局顶点仅用于确定初始位姿。
    auto const scenario = load_scenario(options.input, options.footprint);
    // 规划器的 Pose 不携带 frame_id；场景加载已确保位姿与地图同为 world。
    algorithms::Pose const start{scenario.start_pose.pose.x(),
                                 scenario.start_pose.pose.y(),
                                 scenario.start_pose.pose.z()};
    algorithms::Config config;
    config.max_corner_step = 0.02; // 先尝试 2 cm
    config.safety_margin = 0.02;   // 保持 2 cm 安全余量
    auto const result = timed_call("planner.plan",
                                   [&]
                                   {
                                     return algorithms::plan(scenario.grid,
                                                             start,
                                                             scenario.geometry,
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
