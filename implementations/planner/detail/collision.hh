#pragma once

#include "context.hh"
#include "timing.hh"

namespace algorithms::local_escape_detail
{
double
footprint_clearance(Context const& ctx, Pose const& pose, SearchTiming& timing);

Candidate
extend(Context const& ctx,
       Candidate candidate,
       Motion const& motion,
       SearchTiming& timing);
} // namespace algorithms::local_escape_detail
