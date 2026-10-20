#include "path_smoothing.hh"

#include "backend/osqp.hh"
#include "backend/projected_gradient.hh"
#include "detail/validation.hh"
#include "rerun/debug.hh"

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

  for(std::size_t i = 0; i < points.size(); ++i)
  {
    if(!points[i].allFinite())
    {
      throw std::invalid_argument("path_smoothing: coordinates at point " +
                                  std::to_string(i) + " must be finite");
    }
  }


  detail::ensure_nonnegative_finite(problem.reference_weight,
                                    "reference_weight");
  detail::ensure_nonnegative_finite(problem.smoothness_weight,
                                    "smoothness_weight");
  if(!std::isfinite(problem.tolerance) || problem.tolerance <= 0.0)
  {
    throw std::invalid_argument(
        "path_smoothing: tolerance must be finite and positive");
  }
}

} // namespace


Result
smooth(std::vector<Eigen::Vector2d> const& points, Options const& options)
{
  validate_problem(points, options.problem);
  algorithms::Debug::clear("pg_smooth");
  algorithms::Debug::line("pg_origin", points, {245, 255, 255}, 0.005F, 0.0F);

  auto result = std::visit(
      [&](auto const& solver_options) -> Result
      {
        detail::validate_solver(solver_options);

        return detail::solve_backend(points, options.problem, solver_options);
      },
      options.solver);
  algorithms::Debug::line("pg_smooth",
                          result.points,
                          {255, 122, 133},
                          0.005F,
                          0.0F);
  return result;
}

} // namespace path_smoothing
