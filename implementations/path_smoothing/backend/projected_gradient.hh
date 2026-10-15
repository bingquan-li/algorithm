#include "path_smoothing.hh"


namespace path_smoothing::detail
{

// 后端参数校验：参数目前为空，所以暂时不做任何事情。
void
validate_solver(ProjectedGradientOptions const&);

// 同名重载：通过第三个参数的类型区分后端。
Result
solve_backend(std::vector<Eigen::Vector2d> const& points,
              ProblemOptions const& /* common */,
              ProjectedGradientOptions const& /* solver */);

} // namespace path_smoothing::detail