#include "backend/projected_gradient.hh"
#include "detail/validation.hh"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>



namespace path_smoothing::detail
{

namespace
{

[[noreturn]] void
numerical_error(std::size_t iteration,
                char const* stage,
                bool verbose,
                std::optional<std::size_t> point = std::nullopt)
{
  std::ostringstream message;
  message << "path_smoothing: projected_gradient numerical error at iteration "
          << iteration;
  if(point)
  {
    message << ", point " << *point;
  }
  message << ": non-finite or unrepresentable " << stage;
  if(verbose)
  {
    std::clog << "\nstatus:               numerical error\n"
              << "reason:               " << message.str() << '\n';
  }
  throw std::runtime_error(message.str());
}

// 仅在 verbose 开启时计算诊断代价，不参与求解过程。
long double
log_iteration(std::size_t iteration,
              std::vector<Eigen::Vector2d> const& current,
              std::vector<Eigen::Vector2d> const& origin,
              ProblemOptions const& problem,
              double max_step,
              double step,
              std::size_t clipped,
              ProjectedGradientOptions const& solver,
              double elapsed_seconds)
{
  // 扩大诊断计算的范围，避免有限坐标在平方时产生 double 溢出。
  // 零权重项直接跳过，避免 0 * inf；不让 verbose 改变求解行为。
  long double reference_cost = 0.0L;
  long double smoothness_cost = 0.0L;
  long double violation = 0.0L;
  for(std::size_t i = 0; i < current.size(); ++i)
  {
    Eigen::Matrix<long double, 2, 1> const offset =
        current[i].cast<long double>() - origin[i].cast<long double>();
    if(problem.reference_weight > 0.0)
    {
      reference_cost += offset.squaredNorm();
    }
    long double const distance = solver.shape == RegionShape::Circle
                                     ? std::hypot(offset.x(), offset.y())
                                     : offset.cwiseAbs().maxCoeff();
    violation = std::max(violation, distance - solver.max_displacement);
  }
  if(problem.smoothness_weight > 0.0)
  {
    for(std::size_t i = 1; i + 1 < current.size(); ++i)
    {
      smoothness_cost += (current[i - 1].cast<long double>() -
                          2.0L * current[i].cast<long double>() +
                          current[i + 1].cast<long double>())
                             .squaredNorm();
    }
  }
  reference_cost *= problem.reference_weight;
  smoothness_cost *= problem.smoothness_weight;

  // 在局部流中设置格式，避免改变调用方的日志格式。
  std::ostringstream line;
  line << std::scientific << std::setprecision(6) << std::left << std::setw(8)
       << iteration << std::right << std::setw(16)
       << reference_cost + smoothness_cost << std::setw(16) << reference_cost
       << std::setw(16) << smoothness_cost;
  if(iteration == 0)
  {
    line << std::setw(16) << "N/A" << std::setw(16) << "N/A" << std::setw(10)
         << "N/A";
  }
  else
  {
    line << std::setw(16) << max_step << std::setw(16)
         << static_cast<long double>(max_step) / step << std::setw(10)
         << clipped;
  }
  line << std::setw(16) << violation << std::setw(16) << elapsed_seconds
       << "s\n";
  std::clog << line.str();
  return reference_cost + smoothness_cost;
}

// 这里自己又忘记如何计算梯度了。。。因为自己都没有去想目标函数是什么。。。
// 路径平滑的目标函数：
// J = w_ref * Σ ||p[i] - origin[i]||²
//   + w_smooth * Σ ||p[i-1] - 2*p[i] + p[i+1]||²
// “一个平滑项”是一个数，但它同时依赖三个点，所以可以分别对这三个点求导。
//       梯度描述的是：改变其中某个点时，这个代价会怎样变化。
// 固定一个点看：一个内部点通常会参与三个相邻的平滑项。
// 固定一个平滑项看：这一项会给三个点的梯度带来贡献。
// 求梯度需要明确对哪个变量求导，而不是仅对外层平方求导。

// ! 有一个重要的点，自己以为 J 求导，就是 2*() 这样。。。实际上并不是，J 求导是要求它对每一个 pi 的导数的。。。
std::vector<Eigen::Vector2d>
smooth_gradient(std::vector<Eigen::Vector2d> const& current,
                std::vector<Eigen::Vector2d> const& origin,
                ProblemOptions const& options)
{
  ensure_same_size(current, origin);

  std::size_t const size = current.size();
  // gradient[i] 就是 J 对 p[i] 的梯度！
  std::vector<Eigen::Vector2d> gradient(size);

  // 每个参考项只依赖对应的 p[i]。
  // 括号内部对 p[i] 的导数为单位矩阵，因此得到下面的结果。
  for(std::size_t i = 0; i < size; ++i)
  {
    gradient[i].setZero();
    if(options.reference_weight > 0.0)
    {
      gradient[i] = 2 * options.reference_weight * (current[i] - origin[i]);
    }
  }

  for(std::size_t i = 1; options.smoothness_weight > 0.0 && i + 1 < size; ++i)
  {
    // 当前平滑项：J_i = w_smooth * ||residual||²。
    // 这里自己想的是，每一个 current 会被当作 1,-2,1 来计算三次，因此只需要单独的计算每一个，然后赋给三个梯度就可以了
    // Eigen::Vector2d const radius =
    //     2.0 * options.smoothness_weight *
    //     (current[i - 1] - 2 * current[i] + current[i + 1]);
    // 当前平滑项：J_i = w_smooth * ||residual||²。
    // 这个 J_i 涉及三个 Pi，因此要更新三个对应的梯度
    Eigen::Vector2d const residual =
        current[i - 1] - 2.0 * current[i] + current[i + 1];

    // J_i 对 residual 的梯度。
    Eigen::Vector2d const term_gradient =
        2.0 * options.smoothness_weight * residual;

    // residual 对三个点的导数分别为 I、-2I、I。
    // 按链式法则，将 J_i 对三个点的贡献累加到总梯度。
    gradient[i - 1] += residual;
    gradient[i] -= 2 * residual;
    gradient[i + 1] += residual;
  }

  // 也可以另一种麻烦的做法, 直接计算 J 对每个 pi 的梯度
  // // 以下 size >= 4，各种边界分类不再重叠。
  // double const scale = 2.0 * options.smoothness_weight;
  // for(std::size_t i = 0; i < size; ++i)
  // {
  //   if(i == 0)
  //   {
  //     gradient[i] += scale * (current[0] - 2.0 * current[1] + current[2]);
  //     continue;
  //   }
  //   if(i == 1)
  //   {
  //     gradient[i] +=
  //         scale * (-2.0 * (current[0] - 2.0 * current[1] + current[2]) +
  //                  (current[1] - 2.0 * current[2] + current[3]));
  //     continue;
  //   }
  //   if(i == size - 2)
  //   {
  //     gradient[i] +=
  //         scale * ((current[i - 2] - 2.0 * current[i - 1] + current[i]) -
  //                  2.0 * (current[i - 1] - 2.0 * current[i] + current[i + 1]));
  //     continue;
  //   }
  //   if(i == size - 1)
  //   {
  //     gradient[i] +=
  //         scale * (current[i - 2] - 2.0 * current[i - 1] + current[i]);
  //     continue;
  //   }
  //   gradient[i] +=
  //       scale * ((current[i - 2] - 2.0 * current[i - 1] + current[i]) -
  //                2.0 * (current[i - 1] - 2.0 * current[i] + current[i + 1]) +
  //                (current[i] - 2.0 * current[i + 1] + current[i + 2]));
  // }

  return gradient;
}


Eigen::Vector2d
project_into_region(Eigen::Vector2d const& offset,
                    ProjectedGradientOptions const& solve,
                    double distance)
{
  double const radius = solve.max_displacement;

  if(solve.shape == RegionShape::Circle)
  {
    // return std::max(1.0, solve.max_displacement / offset.norm()) * offset;
    if(distance <= radius)
    {
      return offset; // 圆内或边界上，包括零向量。
    }

    return (radius / distance) * offset;
  }
  // return {std::clamp(offset.x(),
  //                    offset.x() - solve.max_displacement,
  //                    offset.x() + solve.max_displacement),
  //         std::clamp(offset.y(),
  //                    offset.y() - solve.max_displacement,
  //                    offset.y() + solve.max_displacement)};
  return {std::clamp(offset.x(), -radius, radius),
          std::clamp(offset.y(), -radius, radius)};
}
} // namespace


void
validate_solver(ProjectedGradientOptions const& options)
{
  ensure_nonnegative_finite(options.max_displacement, "max_displacement");
  if(options.shape != RegionShape::Circle &&
     options.shape != RegionShape::Square)
  {
    throw std::invalid_argument("path_smoothing: invalid RegionShape");
  }
}

// 同名重载：通过第三个参数的类型区分后端。
Result
solve_backend(std::vector<Eigen::Vector2d> const& origin,
              ProblemOptions const& problem,
              ProjectedGradientOptions const& solver)
{
  using Clock = std::chrono::steady_clock;
  auto const start = solver.verbose ? Clock::now() : Clock::time_point{};
  auto const elapsed_seconds = [&]()
  { return std::chrono::duration<double>(Clock::now() - start).count(); };
  long double final_objective = 0.0L;
  // 按最大迭代次数安排约 20 次周期输出，另保留初始、首次和最终状态。
  // 用商和余数向上取整，避免 max_iterator + 19 溢出。
  std::size_t const log_interval = std::max<std::size_t>(
      1,
      solver.max_iterator / 20 +
          static_cast<std::size_t>(solver.max_iterator % 20 != 0));

  double const lipschitz =
      2.0 * problem.reference_weight + 32.0 * problem.smoothness_weight;

  bool const zero_objective =
      problem.reference_weight == 0.0 && problem.smoothness_weight == 0.0;
  if(!std::isfinite(lipschitz) || (!zero_objective && lipschitz <= 0.0))
  {
    numerical_error(0, "Lipschitz bound", solver.verbose);
  }

  double const step = zero_objective ? 0.0 : 1.0 / lipschitz;
  if(!zero_objective && (!std::isfinite(step) || step <= 0.0))
  {
    numerical_error(0, "step size", solver.verbose);
  }
  std::size_t const size = origin.size();
  std::vector<Eigen::Vector2d> current = origin;
  std::vector<Eigen::Vector2d> next(size);

  Result result;
  result.iterations = solver.max_iterator;
  if(zero_objective)
  {
    result.iterations = 0;
    result.max_step = 0.0;
    result.converged = true;
  }

  if(solver.verbose)
  {
    std::ostringstream line;
    line
        << "-----------------------------------------------------------------\n"
        << "       Projected Gradient - Path Smoothing Solver\n"
        << "-----------------------------------------------------------------"
           "\n\n"
        << "problem:  variables n = " << 2 * size << ", points = " << size
        << '\n'
        << "settings: region = "
        << (solver.shape == RegionShape::Circle ? "circle" : "square")
        << ", fix_endpoints = " << (problem.fix_endpoints ? "on" : "off")
        << '\n'
        << std::scientific << std::setprecision(6)
        << "          reference_weight = " << problem.reference_weight
        << ", smoothness_weight = " << problem.smoothness_weight << '\n'
        << "          max_displacement = " << solver.max_displacement << '\n'
        << "          tolerance = " << problem.tolerance
        << ", step_size = " << step << '\n'
        << "          max_iter = " << solver.max_iterator
        << ", verbose = 1, log_interval = " << log_interval << '\n'
        << "          stopping criterion: max_step < tolerance\n"
        << "          pg_norm = max_step / step_size; clipped = projected free "
           "points\n"
        << "          violation = max region excess (circle: L2, square: "
           "Linf)\n\n"
        << std::left << std::setw(8) << "iter" << std::right << std::setw(16)
        << "objective" << std::setw(16) << "reference" << std::setw(16)
        << "smoothness" << std::setw(16) << "max_step" << std::setw(16)
        << "pg_norm" << std::setw(10) << "clipped" << std::setw(16)
        << "violation" << std::setw(17) << "time" << '\n';
    std::clog << line.str();
    final_objective = log_iteration(0,
                                    current,
                                    origin,
                                    problem,
                                    0.0,
                                    step,
                                    0,
                                    solver,
                                    elapsed_seconds());
  }

  for(std::size_t iter = 0; !zero_objective && iter < solver.max_iterator;
      ++iter)
  {
    std::size_t const iteration = iter + 1;
    auto gradient = smooth_gradient(current, origin, problem);

    double max_step{0.0};
    std::size_t clipped = 0;
    for(std::size_t i = 0; i < size; ++i)
    {
      if(!gradient[i].allFinite())
      {
        numerical_error(iteration, "gradient", solver.verbose, i);
      }
      if(problem.fix_endpoints && (i == 0 || i + 1 == size))
      {
        next[i] = origin[i];
      }
      else
      {
        Eigen::Vector2d offset = current[i] - origin[i] - step * gradient[i];
        if(!offset.allFinite())
        {
          numerical_error(iteration,
                          "offset before projection",
                          solver.verbose,
                          i);
        }
        double const distance = solver.shape == RegionShape::Circle
                                    ? offset.stableNorm()
                                    : offset.cwiseAbs().maxCoeff();
        if(!std::isfinite(distance))
        {
          numerical_error(iteration, "projection distance", solver.verbose, i);
        }
        if(distance > solver.max_displacement)
        {
          ++clipped;
        }
        next[i] = origin[i] + project_into_region(offset, solver, distance);
      }

      if(!next[i].allFinite())
      {
        numerical_error(iteration, "point after projection", solver.verbose, i);
      }
      double const displacement = (next[i] - current[i]).stableNorm();
      if(!std::isfinite(displacement))
      {
        numerical_error(iteration, "displacement", solver.verbose, i);
      }
      max_step = std::max(max_step, displacement);
    }

    current.swap(next);
    result.max_step = max_step;

    bool const converged = max_step < problem.tolerance;
    // 最终轮必须打印，保证摘要中的 objective 对应最终解。
    if(solver.verbose && (iteration == 1 || iteration % log_interval == 0 ||
                          converged || iteration == solver.max_iterator))
    {
      final_objective = log_iteration(iteration,
                                      current,
                                      origin,
                                      problem,
                                      max_step,
                                      step,
                                      clipped,
                                      solver,
                                      elapsed_seconds());
    }

    if(converged)
    {
      result.iterations = iteration;
      result.converged = true;
      break;
    }
  }

  if(solver.verbose)
  {
    char const* stop_reason = "iteration budget exhausted";
    if(zero_objective)
    {
      stop_reason = "zero objective (both weights are zero)";
    }
    else if(result.converged)
    {
      stop_reason = "max_step < tolerance";
    }
    else if(solver.max_iterator == 0)
    {
      stop_reason = "max_iter = 0; no iterations performed";
    }
    std::ostringstream summary;
    summary << '\n'
            << std::scientific << std::setprecision(6)
            << "status:               "
            << (result.converged ? "solved" : "maximum iterations reached")
            << '\n'
            << "stop reason:          " << stop_reason << '\n'
            << "number of iterations: " << result.iterations << '\n'
            << (result.converged ? "optimal objective:    "
                                 : "final objective:      ")
            << final_objective << '\n'
            << "max step:             ";
    if(result.iterations == 0 && !zero_objective)
    {
      summary << "N/A";
    }
    else
    {
      summary << result.max_step;
    }
    summary << "\nprojected grad norm:  ";
    if(zero_objective)
    {
      summary << 0.0;
    }
    else if(result.iterations == 0)
    {
      summary << "N/A";
    }
    else
    {
      summary << static_cast<long double>(result.max_step) / step;
    }
    summary << "\nrun time:             " << elapsed_seconds() << "s\n\n";
    std::clog << summary.str();
  }

  result.points = std::move(current);
  return result;
}

} // namespace path_smoothing::detail
