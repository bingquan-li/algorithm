#include "path_smoothing.hh"

#include "backend/osqp.hh"
#include "backend/projected_gradient.hh"
#include "detail/validation.hh"

namespace path_smoothing
{

namespace
{

void
validate_problem(std::vector<Eigen::Vector2d> const& points,
                 ProblemOptions const& problem)
{
  if(points.size() < 3)
  {
    throw std::invalid_argument(
        "path_smoothing: at least 3 points are required");
  }

  for(auto const& point : points)
  {
    if(!std::isfinite(point.x()) || !std::isfinite(point.y()))
    {
      throw std::invalid_argument(
          "path_smoothing: all coordinates must be finite");
    }
  }

  detail::ensure_nonnegative_finite(problem.max_displacement,
                                    "radius must be finite and non-negative");
  detail::ensure_nonnegative_finite(problem.reference_weight,
                                    "w_ref must be finite and non-negative");
  detail::ensure_nonnegative_finite(problem.smoothness_weight,
                                    "w_smooth must be finite and non-negative");
}

} // namespace


Result
smooth(std::vector<Eigen::Vector2d> const& points, Options const& options)
{
  validate_problem(points, options.problem);

  return std::visit(
      [&](auto const& solver_options) -> Result
      {
        detail::validate_solver(solver_options);

        return detail::solve_backend(points, options.problem, solver_options);
      },
      options.solver);
}

} // namespace path_smoothing