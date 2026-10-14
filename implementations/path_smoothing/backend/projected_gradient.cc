#include "path_smoothing.hh"



namespace path_smoothing::detail
{

void
validate_solver(ProjectedGradientOptions const& options)
{}

// 同名重载：通过第三个参数的类型区分后端。
Result
solve_backend(std::vector<Eigen::Vector2d> const& points,
              Options const& common,
              ProjectedGradientOptions const& solver)
{
  return {};
}

} // namespace path_smoothing::detail