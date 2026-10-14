#pragma once

#include <Eigen/Eigen>

#include <vector>
#include <string_view>


struct LocalHeading
{
  bool available{false};
  bool valid{false};
  // 轨迹在某个位置附近的局部方向
  Eigen::Vector2d direction;
};


struct Moments
{
  std::size_t count{0};
  Eigen::Vector2d mean;
  // x,y 方向上的方差
  double xx{0.0};
  double yy{0.0};
  // xy 的协方差
  double xy{0.0};

  void
  add(Eigen::Vector2d const& point)
  {
    ++count;
    auto const delta = point - mean;
    mean += delta / (static_cast<double>(count));
    auto const delta2 = point - mean;
    xx += delta.x() * delta2.x();
    xy += delta.x() * delta2.y();
    yy += delta.y() * delta2.y();
  }

  bool
  direction(Eigen::Vector2d const& forward, Eigen::Vector2d& out) const
  {
    double const trace = xx + yy;
    double const gap = std::hypot(xx - yy, 2.0 * xy);
    if(!(trace > 0.0) || !std::isfinite(trace) || !std::isfinite(gap) ||
       gap <= 1e-12 * trace)
    {
      return false;
    }
    double const theta = 0.5 * std::atan2(2.0 * xy, xx - yy);
    out = {std::cos(theta), std::sin(theta)};
    if(out.dot(forward) < 0.0)
    {
      out *= -1.0;
    }
    return true;
  }
};

struct Result
{
  // 最后通过检查的原始点，包含该点。
  std::size_t end_index = std::numeric_limits<std::size_t>::max();
  std::size_t first_rejected_index = std::numeric_limits<std::size_t>::max();
  std::size_t point_count = 0;
  bool entire_path_straight = false;
  double length = 0.0;        // 接受前缀的累计弧长。
  Eigen::Vector2d line_point; // 拟合直线经过的质心。
  Eigen::Vector2d direction;  // 单位切向量，沿输入点序。
  Eigen::Vector2d normal;     // direction 逆时针旋转 90 度。
  double heading = 0.0;       // atan2(direction.y, direction.x)。
  double max_distance = 0.0;  // 最后通过检查前缀的指标。
  double max_heading_error = 0.0;
};


// Read x,y from CSV rows, optionally preceded by an x,y header.
// Additional columns are ignored. Throws std::runtime_error on read/format errors.
std::vector<Eigen::Vector2d>
load_path(std::string_view file_path);

std::vector<LocalHeading>
local_headings(std::vector<Eigen::Vector2d> const& path,
               std::vector<double> const& s,
               double window,
               double epsilon);

Result
extract(std::vector<Eigen::Vector2d> const& path);