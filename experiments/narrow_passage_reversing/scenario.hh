#pragma once
#include "footprint.hh"
#include "grid_map.hh"
#include <filesystem>
#include <optional>

namespace narrow_passage
{
struct Options
{
  std::filesystem::path input;
  std::filesystem::path footprint;
  std::optional<std::filesystem::path> blueprint;
  bool viewer = true;
  bool help = false;
};

struct Scenario
{
  algorithms::GridMap<std::uint8_t> grid;
  algorithms::RobotGeometry geometry;  // 配置中的固定车型及后轴局部轮廓。
  algorithms::RearAxlePose start_pose; // 后轴的初始世界位姿。
};

Options
parse_options(int argc, char const* const* argv);
Scenario
load_scenario(std::filesystem::path const& input,
              std::filesystem::path const& footprint_config);
} // namespace narrow_passage
