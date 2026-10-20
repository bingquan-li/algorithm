#include "scenario.hh"
#include "local_escape.hh"
#include "timing.hh"
#include "rerun/debug.hh"
#include "rerun/recording.hh"


#include <iostream>
#include <chrono>

using namespace narrow_passage;

int
main(int argc, char** argv)
{
  try
  {
    auto const options = parse_options(argc, argv);
    if(options.help)
    {
      std::cout << "Usage: narrow_passage_reversing [--input frame.json] "
                   "[--footprint footprint.json] [--blueprint layout.rbl] "
                   "[--no-viewer] [--save FILE.rrd]\n";
      return 0;
    }

    ScopedTimer const total_timer("total (includes optional Rerun lifetime)");
    algorithms::viz::Recording const recording(
        "narrow_passage",
        {.output = options.recording_output,
         .blueprint = options.blueprint,
         .viewer = options.viewer});

    auto const timestamp_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    algorithms::Debug::ScopedContext const context({0, timestamp_ns});
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

    // 可视化在规划节点发生；入口只负责提交完成及录制对象的生命周期。
    algorithms::Debug::flush();

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
