#pragma once

#include <Eigen/Eigen>
#include <filesystem>

namespace algorithms
{
struct Footprint
{
  // 世界坐标，米；按边界顺序，不重复存储末尾首点。
  std::vector<Eigen::Vector2d> vertices;
  // 后轴中心
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  // 朝向，宽端指向窄端
  Eigen::Vector2d forward = Eigen::Vector2d::UnitX();

  double front_extent = 0.0;
  double rear_extent = 0.0;
  double left_extent = 0.0;
  double right_extent = 0.0;

  double min_axis_anisotropy = 0.05;
  double min_end_width_difference = 0.05;
  std::size_t width_samples = 16;
  double relative_tolerance = 1e-8;
  double rear_axle_offset = 0.0;
  // 后轴中心相对车身最后端的纵向偏移，单位 m。
  // 0 表示后轴中心位于车身最后端。
  [[nodiscard]] double
  yaw() const noexcept
  {
    return std::atan2(forward.y(), forward.x());
  }
  [[nodiscard]] Eigen::Vector2d
  left() const noexcept
  {
    return {-forward.y(), forward.x()};
  }
  [[nodiscard]] double
  length() const noexcept
  {
    return front_extent + rear_extent;
  }
  [[nodiscard]] double
  width() const noexcept
  {
    return left_extent + right_extent;
  }
};

struct FootprintOptions
{
  // (lambda_max-lambda_min)/(lambda_max+lambda_min) 的最低要求。
  double min_axis_anisotropy = 0.01;
  // 两端宽度差 / 较宽端宽度 的最低要求。
  double min_end_width_difference = 0.01;
  std::size_t width_samples = 100;
  // 几何检测使用归一化坐标；不是以米为单位的固定阈值。
  double relative_tolerance = 1e-10;
  // 后轴中心距离车身最后端的距离，单位 m。
  double rear_axle_offset = 0.0;
};

// 保留你已有的 JSON 格式：foot_print.coordinate_system/unit/data。
// 缺失坐标系或单位时分别按 world/m 处理；不自动补闭合首点。
[[nodiscard]] std::vector<Eigen::Vector2d>
load_footprint_vertices(std::filesystem::path const& path);

// 适用前提：简单、左右对称、细长、后宽前窄的多边形。
// 不要求恰好八个顶点；不自交及非退化会检查。
// 左右对称和“宽端为后方”是调用方提供的形状语义，不从轮廓证明。
// 方向由面积 PCA 与内部截面宽度估计；方向不明确时报 invalid_argument。
[[nodiscard]] Footprint
make_footprint(std::vector<Eigen::Vector2d> vertices,
               FootprintOptions const& options = {});

[[nodiscard]] Footprint
load_footprint(std::filesystem::path const& path,
               FootprintOptions const& options = {});

// 用于 LineStrip 绘制，返回首尾闭合副本，不改变 Footprint 的存储约定。
[[nodiscard]] std::vector<Eigen::Vector2d>
closed_vertices(Footprint const& footprint);
} // namespace algorithms