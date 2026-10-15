#pragma once

#include <Eigen/Eigen>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
#include <variant>
#include <ostream>


namespace path_smoothing
{

enum class RegionShape : std::uint8_t
{
  Circle,
  Square,
};

struct ProjectedGradientOptions
{
  RegionShape shape{RegionShape::Circle};
  std::size_t max_iterator{1000};
  // 圆的半径 / 方形的半边长，必须有限且非负。
  double max_displacement{0.5};
  /// 向 std::clog 输出初始、首次、最终状态及按 max_iterator 自动间隔的迭代信息。
  bool verbose{false};
};

struct OsqpOptions
{};

// 定义“求解什么问题”，与求解后端无关。
struct ProblemOptions
{
  double reference_weight{1.0};
  double smoothness_weight{20.0};
  /// 最大单点更新量的停止阈值，必须有限且为正。
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
  /// 为真表示 max_step < tolerance，或目标函数恒为零；为假表示耗尽迭代次数。
  bool converged{false};
};


// 放在 Result 所属的命名空间中。
inline std::ostream&
operator<<(std::ostream& os, Result const& result)
{
  os << "Result{"
     << "points=" << result.points.size()
     << ", iterations=" << result.iterations
     << ", converged=" << (result.converged ? "true" : "false");

  if(result.iterations > 0)
  {
    os << ", max_step=" << result.max_step;
  }
  else
  {
    os << ", max_step=N/A";
  }

  return os << '}';
}


Result
smooth(std::vector<Eigen::Vector2d> const& points, Options const& options = {});

} // namespace path_smoothing
