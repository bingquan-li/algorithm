#pragma once

#include <Eigen/Eigen>

#include <variant>


namespace path_smoothing
{

enum class RegionShape : std::uint8_t
{
  Circle,
  Square,
};

// 允许暂时为空，后续分别添加参数。
struct ProjectedGradientOptions
{
  double radius{0.05};
  RegionShape shape{RegionShape::Circle};
};

struct OsqpOptions
{};

using SolverOptions = std::variant<ProjectedGradientOptions, OsqpOptions>;

// 定义“求解什么问题”，与求解后端无关。
struct ProblemOptions
{
  double reference_weight = 1.0;
  double smoothness_weight = 20.0;
  double max_displacement = 0.05;
  bool fix_endpoints = true;
};

// 可选的顶层配置，用于方便统一传参。
struct Options
{
  ProblemOptions problem;

  SolverOptions solver = ProjectedGradientOptions{};
};



struct Result
{
  std::vector<Eigen::Vector2d> points;
  bool converged = false;
};

Result
smooth(std::vector<Eigen::Vector2d> const& points, Options const& options = {});

} // namespace path_smoothing