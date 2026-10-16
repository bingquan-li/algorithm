#pragma once

#include "common.h"


namespace reference_line
{
class ReferencePoint
{
public:
  ReferencePoint() = default;
  ReferencePoint(
      Vec2d xy, double theta, SLPoint sl, double kappa, double dkappa)
    : ReferencePoint(xy, theta, sl, kappa, dkappa, 0.0, 0.0)
  {}
  ReferencePoint(Vec2d xy,
                 double theta,
                 SLPoint sl,
                 double kappa,
                 double dkappa,
                 double tangent_x,
                 double tangent_y)
    : xy_(xy)
    , theta_(theta)
    , sl_(sl)
    , kappa_(kappa)
    , dkappa_(dkappa)
    , tangent_x_(tangent_x)
    , tangent_y_(tangent_y)
  {}
  [[nodiscard]] Vec2d
  xy() const
  {
    return xy_;
  }
  SLPoint
  sl() const
  {
    return sl_;
  }
  double
  kappa() const
  {
    return kappa_;
  }
  double
  dkappa() const
  {
    return dkappa_;
  }
  double
  theta() const
  {
    return theta_;
  }
  double
  tangent_x() const
  {
    return tangent_x_;
  }
  double
  tangent_y() const
  {
    return tangent_y_;
  }

private:
  Vec2d xy_{};
  double theta_{0.0};
  SLPoint sl_;
  double kappa_{0.0};
  double dkappa_{0.0};
  double tangent_x_{0.0};
  double tangent_y_{0.0};
};
} // namespace reference_line