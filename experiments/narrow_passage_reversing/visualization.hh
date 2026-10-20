#pragma once
#include "scenario.hh"
#include "local_escape.hh"

#include <rerun.hpp>

namespace narrow_passage
{
void
log_scene(rerun::RecordingStream const& rec,
          Scenario const& scenario,
          std::uint8_t occupied_threshold);

void
log_result(rerun::RecordingStream const& rec,
           Scenario const& scenario,
           algorithms::Result const& result);

// 一次实验的查看器生命周期、可选布局加载及场景/结果提交。
void
visualize(Scenario const& scenario,
          algorithms::Result const& result,
          std::uint8_t occupied_threshold,
          std::optional<std::filesystem::path> const& blueprint);
} // namespace narrow_passage
