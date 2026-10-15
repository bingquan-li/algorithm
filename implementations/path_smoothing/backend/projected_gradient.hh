#pragma once

#include "path_smoothing.hh"


namespace path_smoothing::detail
{

// 检查位移半径及可行域类型。
void
validate_solver(ProjectedGradientOptions const&);

// 同名重载：通过第三个参数的类型区分后端。
Result
solve_backend(std::vector<Eigen::Vector2d> const& points,
              ProblemOptions const& problem,
              ProjectedGradientOptions const& solver);

} // namespace path_smoothing::detail
