#pragma once

#include "lateral_boundary.hh"

#include <Eigen/Eigen>

#include <vector>
#include <string>
#include <optional>

namespace algorithms
{

// 这里表示的是单个运动
/*
   动作              distance    curvature    rotation
  ━━━━━━━━━━━━━━━━  ━━━━━━━━━━  ━━━━━━━━━━━  ━━━━━━━━━━
   前进直线或圆弧        正值     0 或非零           0
  ────────────────  ──────────  ───────────  ──────────
   后退直线或圆弧        负值     0 或非零           0
  ────────────────  ──────────  ───────────  ──────────
   原地旋转                 0            0        非零
*/
struct Motion
{
  double distance = 0.0; // 有符号弧长，负值后退；旋转时为零
  // k = d(theta) / d(s)  同样的正曲率，前进时，yaw增加，后退时，yaw减少
  double curvature = 0.0;
  double rotation = 0.0; // 原地旋转量，逆时针为正
};


struct Pose
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double yaw{0};
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

// 沿车身纵向展开的左右空间分布表
// 相对于某个位姿和某个计划运动距离，机器人周围
// 的左右障碍距离、扣除车身后的净空，以及由这些信息推导出的曲率建议。
struct BoundaryProfile
{
  // 保存的是纵向位置
  std::vector<double> longitudinal_samples;
  // 中心线到左右障碍的原始距离
  latter_boundary::LateralBoundary boundary;
  // 扣除车身后的左右净空
  // clearance 净空
  std::vector<double> left_clearance;
  std::vector<double> right_clearance;
  // 左右净空沿纵向变化的趋势
  /*
    斜率               含义
    ━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    left_slope > 0     越往车头方向，左侧总体越宽
    ─────────────────  ────────────────────────────
    left_slope < 0     越往车头方向，左侧总体越窄
    ─────────────────  ────────────────────────────
    right_slope > 0    越往车头方向，右侧总体越宽
    ─────────────────  ────────────────────────────
    right_slope < 0    越往车头方向，右侧总体越窄

    这是整个查询范围内的总体趋势，不代表每一小段都单调变化。
  */
  double left_slope = 0.0;
  double right_slope = 0.0;
  // preferred_curvature：根据左右净空推导的建议曲率
  // preferred adj 首选的
  double preferred_curvature = 0.0;
};


struct Result
{
  Status status = Status::no_path;
  BoundaryProfile boundary;
  std::vector<Candidate> candidates;
  std::optional<std::size_t> selected;
};

struct Config
{
  double reverse_length = 1.0;       // 每个后退候选的弧长，m
  double min_reverse_progress = 0.1; // 相对初始朝向的净后退距离，m
  double max_curvature = 1.2;        // d(yaw)/d(有符号弧长)，1/m
  double safety_margin = 0.02;       // 整个 footprint 的余量，m
  double max_corner_step = 0.1;     // 角点每次碰撞检查的最大运动上界，m
  double clearance_cap = 0.3;        // 距离评分截断值，m
  std::vector<double> forward_lengths{0.15, 0.3};
  std::vector<double> rotation_angles{-0.3, -0.15, 0.15, 0.3}; // rad
  bool allow_in_place_rotation = true;
  std::size_t forward_beam_width = 3;
  std::uint8_t occupied_threshold = 1;
};

// footprint 使用以 start.position 为原点、start.yaw 为朝向的车体坐标，单位米。
// 接受简单多边形（可重复首点闭合）；非法输入抛出 invalid_argument。
[[nodiscard]] Result
plan(algorithms::GridMap<std::uint8_t> const& grid,
     Pose const& start,
     std::span<Eigen::Vector2d const> footprint,
     Config const& config = {});

} // namespace algorithms