#include "backend/osqp.hh"

namespace path_smoothing::detail
{

void
validate_solver(OsqpOptions const& options)
{}
Result
solve_backend(std::vector<Eigen::Vector2d> const& /*points*/,
              Options const& /* common */,
              OsqpOptions const& /* solver */)
{
  return {};
}
} // namespace path_smoothing::detail