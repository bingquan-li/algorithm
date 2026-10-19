#pragma once
#include <array>
#include <cstddef>

namespace algorithms
{
struct TimingEntry
{
  double milliseconds = 0.0;
  std::size_t calls = 0;
};

// phase 互不重叠；detail 包含子调用，不能与 phase 或彼此重复相加。
struct SearchStatistics
{
  static constexpr std::array<char const*, 6> phase_names{
      "prepare/validate/initial boundary",
      "start clearance",
      "direct reverse search",
      "rotate + reverse search",
      "forward + reverse search",
      "forward + rotate + reverse beam"};
  static constexpr std::array<char const*, 4> detail_names{
      "compute_boundary_profile",
      "footprint_clearance",
      "extend",
      "evaluate_and_record"};
  std::array<TimingEntry, 6> phases{};
  std::array<TimingEntry, 4> details{};
  std::size_t cell_distance_calls = 0;
  double total_milliseconds = 0.0;
};
} // namespace algorithms
