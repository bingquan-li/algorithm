#include "path_smoothing.hh"
#include "detail/validation.hh"



namespace path_smoothing::detail
{

namespace
{

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
    gradient[i] = 2 * options.reference_weight * (current[i] - origin[i]);
  }

  for(std::size_t i = 1; i + 1 < size; ++i)
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
                    ProjectedGradientOptions const& solve)
{
  if(solve.shape == RegionShape::Circle)
  {
    return std::max(1.0, solve.max_displacement / offset.norm()) * offset;
  }

  // return {std::clamp(offset.x(),
  //                    offset.x() - solve.max_displacement,
  //                    offset.x() + solve.max_displacement),
  //         std::clamp(offset.y(),
  //                    offset.y() - solve.max_displacement,
  //                    offset.y() + solve.max_displacement)};
  return {
      std::clamp(offset.x(), solve.max_displacement, solve.max_displacement),
      std::clamp(offset.y(), solve.max_displacement, solve.max_displacement)};
}
} // namespace


void
validate_solver(ProjectedGradientOptions const& options)
{}

// 同名重载：通过第三个参数的类型区分后端。
Result
solve_backend(std::vector<Eigen::Vector2d> const& origin,
              ProblemOptions const& problem,
              ProjectedGradientOptions const& solver)
{
  double const lipschitz =
      2.0 * problem.reference_weight + 32.0 * problem.smoothness_weight;

  ensure_nonnegative_finite(lipschitz, "lipschitz");

  double const step = 1.0 / lipschitz;
  std::size_t const size = origin.size();
  std::vector<Eigen::Vector2d> current = origin;
  std::vector<Eigen::Vector2d> next(size);

  Result result;
  result.iterations = solver.max_iterator;

  for(std::size_t iter = 0; iter < solver.max_iterator; ++iter)
  {
    auto gradient = smooth_gradient(current, origin, problem);

    double max_step{0.0};
    for(std::size_t i = 0; i < size; ++i)
    {
      if(problem.fix_endpoints && (i == 0 || i + 1 == size))
      {
        next[i] = origin[i];
      }
      else
      {
        Eigen::Vector2d offset = current[i] - origin[i] - step * gradient[i];
        next[i] = origin[i] + project_into_region(offset, solver);
      }
      max_step = std::max(max_step, (next[i] - current[i]).norm());
    }
    current.swap(next);

    result.max_step = std::min(max_step, result.max_step);

    if(max_step < problem.tolerance)
    {
      result.iterations = iter;
      result.converged = true;
      break;
    }
  }

  result.points = std::move(current);
  return result;
}

} // namespace path_smoothing::detail