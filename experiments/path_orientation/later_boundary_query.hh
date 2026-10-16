#pragma once

#include "grid_map.hh"

#include <Eigen/Core>
#include <span>
#include <vector>

struct LateralBoundary
{
  std::vector<double> left_width;
  std::vector<double> right_width;
};

// 中心线点 p[i] = pose + longitudinal_samples[i] * normalized(dir)。
// 纵向偏移和返回距离均为米；负偏移在车后，正偏移在车前，保持输入顺序。
// 左侧为 (-dir.y(), dir.x())，两侧距离均非负，量到首个障碍栅格的边缘。
// cell >= occupied_threshold 为障碍（默认所有非零值）；地图外视作障碍。
// 起点在障碍或地图外时两侧均为 0；空采样返回两个空 vector。
// 射线穿过栅格角点时，保守检查两侧相邻格；其他位置按 floor 分配栅格。
// 这是中心线的几何宽度，未扣除机器人半宽或安全裕量。
// 非有限输入、零方向或零阈值抛出 invalid_argument。
[[nodiscard]] LateralBoundary
build_lateral_boundary(algorithms::GridMap<std::uint8_t> const& grid_map,
                       Eigen::Vector2d const& pose,
                       Eigen::Vector2d const& dir,
                       std::span<double const> longitudinal_samples,
                       std::uint8_t occupied_threshold = 1);
