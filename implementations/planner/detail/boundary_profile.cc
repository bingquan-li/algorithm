#include "boundary_profile.hh"
#include "geometry.hh"


#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace algorithms::local_escape_detail
{
// 按直线行程查询边界，以 k*(travel²/2 + travel*body_x) 拟合侧移；仅用于引导曲率，不证明安全。
BoundaryProfile
compute_boundary_profile(Context const& ctx,
                         Pose const& pose,
                         double travel,
                         SearchTiming& timing)
{
  DetailTimer const detail_timer(timing.details[0]);
  double xmin = ctx.footprint.front().x();
  double xmax = xmin;
  for(auto const& p : ctx.footprint)
  {
    xmin = std::min(xmin, p.x());
    xmax = std::max(xmax, p.x());
  }
  BoundaryProfile result;
  double const begin = xmin + std::min(0.0, travel);
  double const end = xmax + std::max(0.0, travel);

  double const intervals = std::max(1.0, std::ceil((end - begin) / 0.1));
  if(!std::isfinite(intervals) || intervals > 100000)
  {
    throw std::invalid_argument("Local escape: boundary range is too large");
  }
  std::size_t const count = static_cast<std::size_t>(intervals) + 1;
  result.longitudinal_samples.reserve(count);

  for(std::size_t i = 0; i < count; ++i)
  {
    result.longitudinal_samples.push_back(begin +
                                          (end - begin) * i / (count - 1));
  }

  result.boundary =
      latter_boundary::build_lateral_boundary(ctx.grid,
                                              pose.position,
                                              direction(pose),
                                              result.longitudinal_samples,
                                              ctx.config.occupied_threshold);

  double numerator = 0.0;
  double denominator = 0.0;
  double cov_left = 0.0;
  double cov_right = 0.0;
  double variance = 0.0;
  double const mean_s = (begin + end) * 0.5;
  for(std::size_t i = 0; i < count; ++i)
  {
    double const s = result.longitudinal_samples[i];

    double const body_x = std::clamp(s - travel, xmin, xmax);

    auto const [low, high] = lateral_extent(ctx.footprint, body_x);

    double const left_clearance =
        result.boundary.left_width[i] - high - ctx.config.safety_margin;
    double const right_clearance =
        result.boundary.right_width[i] + low - ctx.config.safety_margin;

    result.left_clearance.push_back(left_clearance);
    result.right_clearance.push_back(right_clearance);

    double const centered_s = s - mean_s;
    cov_left += centered_s * left_clearance;
    cov_right += centered_s * right_clearance;
    variance += centered_s * centered_s;
    if(s >= xmin + travel && s <= xmax + travel)
    {
      double const desired_lateral_shift =
          std::clamp(0.5 * (left_clearance - right_clearance), -0.25, 0.25);
      double const lateral_shift_coefficient =
          0.5 * travel * travel + travel * body_x;

      double const clearance_weight =
          1.0 /
          (0.05 + std::max(0.0, std::min(left_clearance, right_clearance)));
      numerator +=
          clearance_weight * lateral_shift_coefficient * desired_lateral_shift;
      denominator += clearance_weight * lateral_shift_coefficient *
                     lateral_shift_coefficient;
    }
  }
  result.left_slope = cov_left / variance;
  result.right_slope = cov_right / variance;
  result.preferred_curvature =
      std::clamp(denominator > epsilon ? numerator / denominator : 0.0,
                 -ctx.config.max_curvature,
                 ctx.config.max_curvature);
  return result;
}

std::vector<double>
curvatures(BoundaryProfile const& boundary, Config const& config)
{
  double const preferred = boundary.preferred_curvature;
  std::vector<double> values{0.0, preferred, preferred * 0.5};

  for(double const factor : {-1.0, -0.5, -0.2, 0.2, 0.5, 1.0})
  {
    values.push_back(factor * config.max_curvature);
  }
  std::ranges::sort(values);
  auto const duplicates = std::ranges::unique(
      values,
      [](double a, double b) { return std::abs(a - b) < epsilon; });
  values.erase(duplicates.begin(), duplicates.end());

  std::ranges::stable_sort(
      values,
      [preferred](double a, double b)
      { return std::abs(a - preferred) < std::abs(b - preferred); });
  return values;
}
} // namespace algorithms::local_escape_detail
