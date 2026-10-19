#pragma once

#include "footprint.hh"
#include <filesystem>

namespace narrow_passage
{
// 实验场景格式 foot_print：读取 world/m 轮廓，保留首点并统一逆时针。
[[nodiscard]] algorithms::Footprint
load_footprint(std::filesystem::path const& path);

// 实验配置格式：显式尺寸及可选 data；加载时将顶点转换到后轴坐标系。
[[nodiscard]] algorithms::RobotGeometry
load_robot_geometry(std::filesystem::path const& path);

// 实验用对应顶点配准，返回观测坐标系下的后轴位姿。
// 首点必须对应，最大残差默认 1 mm，不尝试循环移位、缩放或镜像。
[[nodiscard]] algorithms::RearAxlePose
estimate_footprint_pose(algorithms::RobotGeometry const& model,
                        algorithms::Footprint const& observed,
                        double max_vertex_error = 0.001);
} // namespace narrow_passage
