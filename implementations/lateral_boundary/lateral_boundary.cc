#include "lateral_boundary.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
double
ray_distance(algorithms::GridMap<std::uint8_t> const& grid_map,
             Eigen::Vector2d const& point,
             Eigen::Vector2d const& direction,
             std::uint8_t occupied_threshold)
{
  auto const& geometry = grid_map.geometry();
  Eigen::Vector2d const start =
      (point - Eigen::Vector2d(geometry.origin_x, geometry.origin_y)) /
      geometry.resolution;

  // 先检查浮点坐标，再转整数，避免地图外坐标转换溢出。
  if(!start.allFinite() || start.x() < 0.0 || start.y() < 0.0 ||
     start.x() >= static_cast<double>(geometry.width) ||
     start.y() >= static_cast<double>(geometry.height))
  {
    return 0.0;
  }

  int x = static_cast<int>(std::floor(start.x()));
  int y = static_cast<int>(std::floor(start.y()));
  auto const blocked = [&](int cx, int cy)
  {
    return !grid_map.contains(cx, cy) ||
           grid_map(static_cast<std::size_t>(cx),
                    static_cast<std::size_t>(cy)) >= occupied_threshold;
  };
  if(blocked(x, y))
  {
    return 0.0;
  }

  // DDA 按射线与网格线的交点前进，不用固定步长试探，避免跳过薄障碍。
  // t 的单位为栅格分辨率，direction 是世界坐标系中的单位向量。
  double const infinity = std::numeric_limits<double>::infinity();

  // 布尔值转换为整数：根据方向来判断搜索的方向
  int const step_x = static_cast<int>(direction.x() > 0.0) -
                     static_cast<int>(direction.x() < 0.0);
  int const step_y = static_cast<int>(direction.y() > 0.0) -
                     static_cast<int>(direction.y() < 0.0);

  //  delta_x 和 delta_y：相邻同类交点之间相隔多少参数
  // 如果垂直或者平行，就不会发生
  double const delta_x = step_x == 0 ? infinity : 1.0 / std::abs(direction.x());
  double const delta_y = step_y == 0 ? infinity : 1.0 / std::abs(direction.y());

  /*
    delta_x	    相邻竖线交点之间的参数间隔	    固定
    next_x	    从起点算，到下一条竖线的参数值	不断累加
    delta_y	    相邻横线交点之间的参数间隔	    固定
    next_y	    从起点算，到下一条横线的参数值	不断累加

    next_x、next_y 始终是相对于起点的累计参数，不是相对于当前格子的剩余距离。
  */
  // 第一次碰到网格线的位置
  double next_x = step_x == 0
                      ? infinity
                      // 分向左运动还是向右运动
                      : (step_x > 0 ? x + 1.0 - start.x() : start.x() - x) /
                            std::abs(direction.x());
  double next_y = step_y == 0
                      ? infinity
                      : (step_y > 0 ? y + 1.0 - start.y() : start.y() - y) /
                            std::abs(direction.y());

  // x 表示竖线,    y 表示横线
  // 虽然 x 是横轴, y是纵轴
  for(;;)
  {
    // 循环里选择最近的边界事件
    double const distance = std::min(next_x, next_y);
    double const tolerance =
        16.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, distance);
    bool const cross_x = next_x - distance <= tolerance;
    bool const cross_y = next_y - distance <= tolerance;

    // 对于过角点的地方，会额外检查两个侧方的格子
    if(cross_x && cross_y)
    {
      // 同时越过两条网格线时，也检查仅在角点接触的两个侧格。
      if(blocked(x + step_x, y) || blocked(x, y + step_y))
      {
        return distance * geometry.resolution;
      }
    }
    // 计算下一个格子
    if(cross_x)
    {
      x += step_x;
      next_x += delta_x;
    }
    if(cross_y)
    {
      y += step_y;
      next_y += delta_y;
    }

    // 判断下一个格子是否会碰撞
    if(blocked(x, y))
    {
      return distance * geometry.resolution;
    }
  }
}
} // namespace

namespace algorithms::latter_boundary
{

LateralBoundary
build_lateral_boundary(algorithms::GridMap<std::uint8_t> const& grid_map,
                       Eigen::Vector2d const& pose,
                       Eigen::Vector2d const& dir,
                       std::span<double const> longitudinal_samples,
                       std::uint8_t occupied_threshold)
{
  // coefficient-wise 逐元素操作，逐元素取绝对值
  // maxCoeff() 返回最大元素，无穷范数
  if(!pose.allFinite() || !dir.allFinite() ||
     dir.cwiseAbs().maxCoeff() == 0.0 || occupied_threshold == 0)
  {
    throw std::invalid_argument(
        "Lateral boundary: invalid pose, direction or threshold");
  }
  auto const& geometry = grid_map.geometry();
  if(geometry.width >
         static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
     geometry.height >
         static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument(
        "Lateral boundary: map dimensions exceed int range");
  }

  // 先缩放，避免非常大或非常小的方向向量在求模时溢出/下溢。
  Eigen::Vector2d const forward =
      (dir / dir.cwiseAbs().maxCoeff()).normalized();
  Eigen::Vector2d const left(-forward.y(), forward.x());
  LateralBoundary result;
  result.left_width.reserve(longitudinal_samples.size());
  result.right_width.reserve(longitudinal_samples.size());
  result.left_points.reserve(longitudinal_samples.size());
  result.right_points.reserve(longitudinal_samples.size());
  for(double const s : longitudinal_samples)
  {
    Eigen::Vector2d const point = pose + s * forward;
    if(!std::isfinite(s) || !point.allFinite())
    {
      throw std::invalid_argument(
          "Lateral boundary: non-finite centerline point");
    }
    double const left_distance =
        ray_distance(grid_map, point, left, occupied_threshold);

    double const right_distance =
        ray_distance(grid_map, point, -left, occupied_threshold);

    result.left_width.push_back(left_distance);
    result.right_width.push_back(right_distance);

    result.left_points.emplace_back(point + left * left_distance);
    result.right_points.emplace_back(point - left * right_distance);
  }
  return result;
}
} // namespace algorithms::latter_boundary