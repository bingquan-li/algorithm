#pragma once

#include "Eigen/Eigen"
#include <Eigen/Core>
#include "reference_point.h"
#include "config.h"
#include "boundary.h"

#include "grid_map.hh"

#include <vector>

namespace reference_line
{

class ReferenceLine
{
public:
  ReferenceLine(Config* config)
    : config_(config)
  {}

  bool
  create_new_line(algorithms::GridMap<std::uint8_t> const& grid_map,
                  std::vector<Eigen::Vector2d> const& global_path,
                  Eigen::Vector2d const& pose,
                  Eigen::Vector2d const& dir,
                  double resample_spacing = 0.2,
                  double min_length = 0.2,
                  bool smooth_corners = false);

  bool
  create_new_line(std::vector<Eigen::Vector2d> const& global_path,
                  Eigen::Vector2d const& dir,
                  double resample_spacing = 0.2,
                  double min_length = 0.2,
                  bool smooth_corners = false);
  bool
  create_new_line(std::vector<Eigen::Vector3d> const& global_path,
                  double resample_spacing = 0.2,
                  double max_length = 0.2,
                  bool smooth_corners = false);


  bool
  xy_to_sl(Vec2d const& xy_point,
           SLPoint* sl_point,
           double warm_start_s = -1.0) const;
  bool
  sl_to_xy(SLPoint const& sl_point, Vec2d* xy_point) const;
  template <class XYPoint>
  bool
  xy_to_sl(XYPoint const& xy, SLPoint* const sl_point) const
  {
    return xy_to_sl(Vec2d(xy.x(), xy.y()), sl_point);
  }

  [[nodiscard]] ReferencePoint
  get_reference_point(double s) const;
  [[nodiscard]] InterpolatedIndex
  get_index_from_s(double s) const;
  [[nodiscard]] ReferencePoint
  interpolate_with_matched_index(ReferencePoint const& p0,
                                 double s0,
                                 ReferencePoint const& p1,
                                 double s1,
                                 InterpolatedIndex const& index) const;
  [[nodiscard]] double
  length() const
  {
    return length_;
  }
  [[nodiscard]] SLBoundPoint
  get_bound_point_by_s(double s) const
  {
    return geometry_bound_.get_bound_point_by_s(s);
  }

  // 未叠加本周期请求的纯几何边界。调用方负责求交并自己持有结果，
  // 这里不能就地收窄：参考线跨周期复用，就地改会让收窄逐周期累积。
  [[nodiscard]] SLBoundary
  boundary() const
  {
    return geometry_bound_;
  }

  bool
  update_reference_line(Vec2d const& cur_pose);


private:
  void
  build_lateral_boundary(algorithms::GridMap<std::uint8_t> const& grid_map,
                         Eigen::Vector2d const& pose,
                         Eigen::Vector2d const& dir);
  void
  build_lateral_boundary();
  bool
  extract_distinct_points(std::vector<Eigen::Vector3d> const& origin_path,
                          std::vector<Eigen::Vector2d>& sample_path);
  bool
  cumulative_lengths();
  bool
  build_reference_points(std::size_t start_idx = 0);

  struct ReferenceSample
  {
    double x{0.0};
    double y{0.0};
    double dx{0.0};
    double dy{0.0};
    double ddx{0.0};
    double ddy{0.0};
  };
  double
  get_max_curvature(std::vector<Eigen::Vector2d> const& origin_points);

  std::vector<Eigen::Vector2d>
  smoothing_gradient(std::vector<Eigen::Vector2d> const& points,
                     std::vector<Eigen::Vector2d> const& origin_points);
  [[nodiscard]] ReferenceSample
  sample_reference(std::size_t segment, double ratio) const;

  std::vector<Eigen::Vector2d>
  smooth_path(std::vector<Eigen::Vector2d> const& origin_points);
  [[nodiscard]] Eigen::Vector2d
  project_into_region(Eigen::Vector2d offset, double radius) const;
  struct Evaluation
  {
    double value;
    double first_derivative;
    double second_derivative;
  };
  static Evaluation
  evaluate_hermite(double first,
                   double last,
                   double first_d,
                   double last_d,
                   double span,
                   double ratio);

  bool
  debug();

  std::vector<ReferencePoint> reference_points_;

  std::vector<double> accumulated_s_;
  std::vector<Eigen::Vector2d> smoothed_path_;
  double length_{0.0};
  int num_points_{};
  int num_sample_points_ = 0;
  Config* config_;

  SLBoundary geometry_bound_;
  std::vector<double> left_width_;
  std::vector<double> right_width_;

  // segment
  bool truncated_{false};
  std::size_t truncated_index_{0};
  bool valided_{true};
  // debug
  // 这里显示，当前的 sl boundary
  // 笛卡尔下显示
  std::vector<Eigen::Vector3d> left_boundary_sl_;
  std::vector<Eigen::Vector3d> right_boundary_sl_;
  std::vector<Eigen::Vector3d> center_line_;
};

} // namespace reference_line
