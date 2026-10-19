#pragma once


#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace algorithms
{

using Point2d = Eigen::Vector2d;
using Polygon2d = std::vector<Point2d>;
using Pose2d = Eigen::Vector3d;

// 后轴坐标系相对于目标坐标系的位姿。
// 后轴坐标系：原点为后轴中心，x 向前，y 向左。
struct RearAxlePose
{
  std::string frame_id;
  Pose2d pose;
};

struct Bounds2d
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
};

// 车体沿自身坐标轴，相对于后轴中心的边界距离。
// 此表示假设后轴中心位于车体包围盒内。
struct BodyExtents
{
  double front_m;
  double rear_m;
  double left_m;
  double right_m;
};

class Footprint
{
public:
  // 校验简单非退化多边形，移除闭合首点，保留起点并统一为逆时针。
  Footprint(std::string frame_id, Polygon2d vertices);

  [[nodiscard]] std::string const&
  frame_id() const noexcept
  {
    return frame_id_;
  }

  [[nodiscard]] Polygon2d const&
  vertices() const noexcept
  {
    return vertices_;
  }

  // 当前坐标系下的轴对齐包围盒。
  // 当 frame_id 为 map 时，它不是车体方向的前后左右边界。
  [[nodiscard]] Bounds2d
  bounds() const
  {
    auto const& first = vertices_.front();
    Bounds2d b{first.x(), first.x(), first.y(), first.y()};

    for(auto const& p : vertices_)
    {
      b.min_x = std::min(p.x(), b.min_x);
      b.max_x = std::max(p.x(), b.max_x);
      b.min_y = std::min(p.y(), b.min_y);
      b.max_y = std::max(p.y(), b.max_y);
    }

    return b;
  }

private:
  std::string frame_id_;
  Polygon2d vertices_;
};

class RobotGeometry
{
public:
  // local_outline 若提供，必须以后轴中心为原点，
  // x 向前、y 向左，描述与 extents 相同的车体状态。
  explicit RobotGeometry(BodyExtents extents,
                         std::optional<Polygon2d> local_outline = std::nullopt)
    : extents_(extents)
  {
    ValidateExtent(extents.front_m);
    ValidateExtent(extents.rear_m);
    ValidateExtent(extents.left_m);
    ValidateExtent(extents.right_m);

    if(extents.front_m + extents.rear_m <= 0.0 ||
       extents.left_m + extents.right_m <= 0.0)
    {
      throw std::invalid_argument("Degenerate body dimensions");
    }

    if(local_outline)
    {
      local_footprint_.emplace("rear_axle", std::move(*local_outline));

      // 两份数据同时存在时，检查一致性，避免静默冲突。
      auto const b = local_footprint_->bounds();
      constexpr double tolerance_m = 1e-6;

      if(std::abs(b.max_x - extents.front_m) > tolerance_m ||
         std::abs(b.min_x + extents.rear_m) > tolerance_m ||
         std::abs(b.max_y - extents.left_m) > tolerance_m ||
         std::abs(b.min_y + extents.right_m) > tolerance_m)
      {
        throw std::invalid_argument("Local outline and body extents disagree");
      }
    }
  }

  [[nodiscard]] BodyExtents const&
  extents() const noexcept
  {
    return extents_;
  }

  [[nodiscard]] std::optional<Footprint> const&
  local_footprint() const noexcept
  {
    return local_footprint_;
  }

  // 没有完整局部轮廓时，返回未知，不擅自生成矩形替代。
  [[nodiscard]] std::optional<Footprint>
  footprint_at(RearAxlePose const& rear_axle_pose) const
  {
    if(!local_footprint_)
    {
      return std::nullopt;
    }

    auto const& pose = rear_axle_pose.pose;
    if(!std::isfinite(pose.x()) || !std::isfinite(pose.y()) ||
       !std::isfinite(pose.z()))
    {
      throw std::invalid_argument("Non-finite rear axle pose");
    }

    double const c = std::cos(pose.z());
    double const s = std::sin(pose.z());

    Polygon2d transformed;
    transformed.reserve(local_footprint_->vertices().size());

    for(auto const& p : local_footprint_->vertices())
    {
      transformed.emplace_back(pose.x() + c * p.x() - s * p.y(),
                               pose.y() + s * p.x() + c * p.y());
    }

    return Footprint{rear_axle_pose.frame_id, std::move(transformed)};
  }

private:
  static void
  ValidateExtent(double value)
  {
    if(!std::isfinite(value) || value < 0.0)
    {
      throw std::invalid_argument("Invalid body extent");
    }
  }

  BodyExtents extents_;
  std::optional<Footprint> local_footprint_;
};

// 当前观测或计算得到的几何状态。
// rear_axle_pose 若存在，应与 footprint 使用相同坐标系。
struct RobotGeometryState
{
  Footprint footprint;
  std::optional<RearAxlePose> rear_axle_pose;

  // 空指针表示车型参数未知；多个状态可共享同一只读模型。
  std::shared_ptr<RobotGeometry const> model;
};

// 安全策略独立于实际几何。
struct CollisionGeometryConfig
{
  double padding_m{0.0};
};

} // namespace algorithms
