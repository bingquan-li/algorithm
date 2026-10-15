#pragma once

#include <Eigen/Eigen>


#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>

namespace path_smoothing::detail
{

inline void
ensure_nonnegative_finite(double value, char const* name)
{
  if(!std::isfinite(value) || value < 0.0)
  {
    throw std::invalid_argument(std::string{name} +
                                " must be finite and nonnegative");
  }
}

inline void
ensure_same_size(std::vector<Eigen::Vector2d> const& a,
                 std::vector<Eigen::Vector2d> const& b)
{
  if(a.size() != b.size())
  {
    throw std::invalid_argument(
        "path_smoother: points and origin_points must have equal size");
  }
}

} // namespace path_smoothing::detail