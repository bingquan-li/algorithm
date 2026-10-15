#pragma once

#include "path_smoothing.hh"

namespace path_smoothing::detail
{
void
validate_solver(OsqpOptions const&);

Result
solve_backend(std::vector<Eigen::Vector2d> const& points,
              ProblemOptions const& problem,
              OsqpOptions const& solver);
} // namespace path_smoothing::detail