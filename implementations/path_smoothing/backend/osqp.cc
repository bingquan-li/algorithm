#include "backend/osqp.hh"

#include <stdexcept>

namespace path_smoothing::detail
{

void
validate_solver(OsqpOptions const& options)
{}
Result
solve_backend(std::vector<Eigen::Vector2d> const& /*points*/,
              ProblemOptions const& /* problem */,
              OsqpOptions const& /* solver */)
{
  throw std::logic_error("path_smoothing: OSQP backend is not implemented");
}
} // namespace path_smoothing::detail
