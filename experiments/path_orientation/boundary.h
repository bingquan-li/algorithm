#pragma once

#include <vector>
#include <utility>
namespace reference_line
{

struct SLBoundPoint
{
  double s{0.};
  double l_upper{0.};
  double l_lower{0.};

  double
  width() const
  {
    return l_upper - l_lower;
  }
};

class SLBoundary
{
public:
  SLBoundary() = default;

  SLBoundary(std::vector<SLBoundPoint>&& boundary)
    : boundary_(std::move(boundary))
  {}

  void
  set_boundary(std::vector<SLBoundPoint>&& boundary)
  {
    boundary_ = std::move(boundary);
  }

  void
  set_delta_s(double delta_s)
  {
    delta_s_ = delta_s;
  }

  double
  delta_s() const
  {
    return delta_s_;
  }
  double
  get_lower_bound_by_s(double s) const;
  double
  get_upper_bound_by_s(double s) const;

  SLBoundPoint
  get_bound_point_by_s(double s) const;

  std::vector<SLBoundPoint> const&
  get_boundary() const
  {
    return boundary_;
  }

  [[nodiscard]] bool
  empty() const
  {
    return boundary_.empty();
  }

  // 与一条等宽走廊求交。横向量来源多且含义不同（几何转弯半径、
  // 场景下发的路宽、config 上限），全部收进同一份剖面里。
  [[nodiscard]] SLBoundary
  intersect(double l_lower, double l_upper) const;

  // 把每一步的上下界放宽到包含 l。走廊收窄时机器人可能已经在
  // 走廊外，把它排除在外会让首层边无解。
  void
  include(double min_l, double max_l);

  // 最窄处的走廊宽度。负值表示某个 s 上上下界已经翻转。
  [[nodiscard]] double
  min_width() const;

private:
  [[nodiscard]] double
  get_interpolated_s_width(double s, bool upper) const;

  std::vector<SLBoundPoint> boundary_;
  double delta_s_{0.2};
};

} // namespace reference_line