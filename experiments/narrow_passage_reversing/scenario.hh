#pragma once
#include "footprint.hh"
#include "local_escape.hh"
#include <filesystem>
#include <optional>

namespace narrow_passage
{
struct Options
{
  std::filesystem::path input;
  std::optional<std::filesystem::path> blueprint;
  bool viewer = true;
  bool help = false;
};

struct Scenario
{
  algorithms::GridMap<std::uint8_t> grid;
  algorithms::Footprint world_footprint;
  algorithms::Pose start;
  std::vector<Eigen::Vector2d> body_footprint;
};

Options
parse_options(int argc, char const* const* argv);
Scenario
load_scenario(std::filesystem::path const& input);
} // namespace narrow_passage
