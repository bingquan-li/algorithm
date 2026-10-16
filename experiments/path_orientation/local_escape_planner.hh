#pragma once

#include "later_boundary_query.hh"

#include <optional>
#include <string>

namespace local_escape
{
struct Pose
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double yaw = 0.0;
};

struct Config
{
  double reverse_length = 0.6;       // 每个后退候选的弧长，m
  double min_reverse_progress = 0.1; // 相对初始朝向的净后退距离，m
  double max_curvature = 1.2;        // d(yaw)/d(有符号弧长)，1/m
  double safety_margin = 0.02;       // 整个 footprint 的余量，m
  double max_corner_step = 0.02;     // 角点每次碰撞检查的最大运动上界，m
  double clearance_cap = 0.3;        // 距离评分截断值，m
  std::vector<double> forward_lengths{0.15, 0.3};
  std::vector<double> rotation_angles{-0.3, -0.15, 0.15, 0.3}; // rad
  bool allow_in_place_rotation = true;
  std::size_t forward_beam_width = 3;
  std::uint8_t occupied_threshold = 1;
};

struct Motion
{
  double distance = 0.0; // 有符号弧长，负值后退；旋转时为零
  double curvature = 0.0;
  double rotation = 0.0; // 原地旋转量，逆时针为正
};

struct BoundaryProfile
{
  std::vector<double> longitudinal_samples;
  LateralBoundary widths;
  std::vector<double> left_clearance;
  std::vector<double> right_clearance;
  double left_slope = 0.0; // 沿车头方向的余量变化率
  double right_slope = 0.0;
  double preferred_curvature = 0.0;
};

struct Candidate
{
  std::vector<Pose> poses; // 起点及已通过连续区间检查的各端点
  std::vector<Motion> motions;
  std::optional<Pose> rejected_pose; // 首个未通过碰撞/安全余量检查的位姿
  bool feasible = false;
  double min_clearance = 0.0;
  // 含区间运动误差后的保守下界，最大为 clearance_cap
  double reverse_progress = 0.0;
  double score = 0.0;
  std::string reason;
};

enum class Status : std::uint8_t
{
  success,
  start_blocked,
  no_path
};

struct Result
{
  Status status = Status::no_path;
  BoundaryProfile boundary;
  std::vector<Candidate> candidates;
  std::optional<std::size_t> selected;
};

// footprint 为简单多边形，车体坐标 x 向前、y 向左，原点必须是差速旋转中心。
// 本模块输出静态地图上的几何轨迹；执行器仍需做速度规划并在执行中重新规划。
// 不可行时 selected 为空，不把失败候选的安全前缀当成可执行脱困轨迹。
[[nodiscard]] Result
plan(algorithms::GridMap<std::uint8_t> const& grid,
     Pose const& start,
     std::span<Eigen::Vector2d const> footprint,
     Config const& config = {});

// 多边形与完整障碍格/地图边缘的欧氏距离（截断到 cap），内部障碍也会检测。
// 前置条件：有效地图、简单多边形、有限位姿、cap > 0。
[[nodiscard]] double
footprint_clearance(algorithms::GridMap<std::uint8_t> const& grid,
                    Pose const& pose,
                    std::span<Eigen::Vector2d const> footprint,
                    double cap,
                    std::uint8_t occupied_threshold = 1);

[[nodiscard]] std::vector<Eigen::Vector2d>
world_footprint(Pose const& pose, std::span<Eigen::Vector2d const> footprint);
} // namespace local_escape
