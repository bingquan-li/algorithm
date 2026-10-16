#pragma once

#include <cstddef>

namespace reference_line
{
// 从 DpConfig 提取的建系参数，默认值与规划器保持一致。
struct Config
{
  double max_reference_line_length{20};
  double upper_bound{0.35};
  double lower_bound{-0.35};
  double w_ref{2.0};
  double w_smooth{2000.0};
  std::size_t max_iter{1000};
  bool fix_end{true};
  double radius{0.1};
  double tolerance{1e-8};
  double rate{0.3};
};
} // namespace reference_line
