#include "path_smoothing.hh"

#include "backend/osqp.hh"
#include "backend/projected_gradient.hh"


namespace path_smoothing
{

namespace
{

void
ensure_nonnegative_finite(double value, char const* name)
{
  if(!std::isfinite(value) || value < 0.0)
  {
    throw std::invalid_argument(name);
  }
}

void
validate_common(std::vector<Eigen::Vector2d> const& points,
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

  ensure_nonnegative_finite(problem.max_displacement,
                            "radius must be finite and non-negative");
  ensure_nonnegative_finite(problem.reference_weight,
                            "w_ref must be finite and non-negative");
  ensure_nonnegative_finite(problem.smoothness_weight,
                            "w_smooth must be finite and non-negative");
}

} // namespace

Result
smooth(std::vector<Eigen::Vector2d> const& points, Options const& options)
{
  validate_common(points, options.problem);

  return std::visit(
      [&](auto const& solver_options) -> Result
      {
        detail::validate_solver(solver_options);

        return detail::solve_backend(points, options, solver_options);
      },
      options.solver);
}

} // namespace path_smoothing