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
  RegionShape shape{RegionShape::Circle};
  std::size_t max_iterator{1000};
  // 最大位移
  double max_displacement{0.05};
};

struct OsqpOptions
{};

// 定义“求解什么问题”，与求解后端无关。
struct ProblemOptions
{
  double reference_weight{1.0};
  double smoothness_weight{20.0};
  double tolerance{1e-8};
  bool fix_endpoints = true;
};


using SolverOptions = std::variant<ProjectedGradientOptions, OsqpOptions>;

// 可选的顶层配置，用于方便统一传参。
struct Options
{
  ProblemOptions problem;
  SolverOptions solver{ProjectedGradientOptions{}};
};



struct Result
{
  std::vector<Eigen::Vector2d> points;
  std::size_t iterations{0};
  /// 最后一轮的最大单点位移，即收敛判据的实测值。
  double max_step{std::numeric_limits<double>::max()};
  /// 为真表示 max_step 已降到 tolerance 以下；为假表示耗尽了 max_iter。
  bool converged{false};
};

Result
smooth(std::vector<Eigen::Vector2d> const& points, Options const& options = {});

} // namespace path_smoothing