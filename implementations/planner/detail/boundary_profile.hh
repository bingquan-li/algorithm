#pragma once

#include "context.hh"
#include "timing.hh"

namespace algorithms::local_escape_detail
{
BoundaryProfile
compute_boundary_profile(Context const& ctx,
                         Pose const& pose,
                         double travel,
                         SearchTiming& timing);

std::vector<double>
curvatures(BoundaryProfile const& boundary, Config const& config);
} // namespace algorithms::local_escape_detail
