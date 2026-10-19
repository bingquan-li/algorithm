#include "local_escape.hh"

namespace algorithms
{

constexpr double epsilon = 1e-10;

Eigen::Vector2d
direction(Pose const& pose)
{
  return {std::cos(pose.yaw), std::sin(pose.yaw)};
}

/*
单词	表示
width	宽度，通常是一个非负标量
extent	占据范围，可以包含最小值和最大值
offset	相对于参考位置的偏移量
boundary	边界位置或边界几何
*/
// extent：范围、延伸程度，表示一个物体在某个方向上占据的空间范围。
std::pair<double, double>
lateral_extent(std::span<Eigen::Vector2d> footprint, double x)
{
  double low = std::numeric_limits<double>::infinity();
  double high = -low;
  // 遍历机器人的每一条边
  for(std::size_t i = 0; i < footprint.size(); ++i)
  {
    // 每次取两个相邻顶点
    auto const& a = footprint[i];
    auto const& b = footprint[(i + 1) % footprint.size()];
    // 排除不可能相交的点
    if(x < std::min(a.x(), b.x()) || x > std::max(a.x(), b.x()))
    {
      continue;
    }
    // 如果是平行的 ==> 其实是重合了。。。
    if(a.x() == b.x())
    {
      low = std::min({low, a.y(), b.y()});
      high = std::max({high, a.y(), b.y()});
    }
    else
    {
      // 线性插值求交点，自己以为是求解一次函数方程，然后计算交点。。。其实用线性插值理解会更好
      double const y = a.y() + (x - a.x()) * (b.y() - a.y()) / (b.x() - a.x());
      low = std::min(low, y);
      high = std::max(high, y);
    }
  }
  return {low, high};
}


// 构建左右净空分布，并计算边界变化趋势和建议曲率
// 在机器人导航、路径规划和几何处理中，profile 通常表示某个物理量沿特定维度的分布或变化情况。
/*
场景	中文含义	示例
日常英语	侧面、轮廓	face profile（面部侧面轮廓）
用户系统	个人资料、配置档案	UserProfile
数据分析	特征描述、分布情况	DataProfile
机器人规划	沿某个维度的变化分布	VelocityProfile
软件性能分析	性能剖析	CPU profiling
*/
// 传入 travel 通常表示行程，而你这里真正使用的是沿初始车身 X 轴的有符号纵向位移。
BoundaryProfile
compute_boundary_profile(algorithms::GridMap<std::uint8_t> const& grid,
                         Pose const& pose,
                         std::span<Eigen::Vector2d> footprint,
                         Config const& config,
                         double travel)
{
  // 这里的 footprint 按前文约定，使用车身坐标系。并非是在 gridmap 中的坐标！
  // 纵向查询范围，机器人自身坐标系
  double xmin = footprint.front().x();
  double xmax = xmin;
  for(auto const& p : footprint)
  {
    xmin = std::min(xmin, p.x());
    xmax = std::max(xmax, p.x());
  }
  BoundaryProfile result;
  // 当前车身及运动后车身所在区域都查询，不能只查询当前车长。
  // 这个范围只按直线平移构造，不等于旋转运动的完整扫掠包络。
  double const begin = xmin + std::min(0.0, travel);
  double const end = xmax + std::max(0.0, travel);

  std::size_t const count = (begin - end) / 0.1 + 1;

  for(std::size_t i = 0; i < count; ++i)
  {
    result.longitudinal_samples.push_back(begin +
                                          (end - begin) * i / (count - 1));
  }

  result.boundary =
      latter_boundary::build_lateral_boundary(grid,
                                              pose.position,
                                              direction(pose),
                                              result.longitudinal_samples,
                                              config.occupied_threshold);

  //  分子。
  double numerator = 0.0;
  // 分母
  double denominator = 0.0;
  // cov：covariance 的缩写，表示协方差。两个变量相对于各自均值的变化，是否同向。
  double cov_left = 0.0;
  double cov_right = 0.0;
  // variance：表示方差。自身有多分散
  double variance = 0.0;
  // 由于采样均匀且包含两端，所以这个值就是所有 \(s_i\) 的平均值。
  double const mean_s = (begin + end) * 0.5;
  for(std::size_t i = 0; i < count; ++i)
  {
    double const s = result.longitudinal_samples[i];

    double const body_x = std::clamp(s - travel, xmin, xmax);

    // lateral_extent: 求车身在指定纵向位置 x 处的横向范围
    // 用直线 \(X=x\) 切车身多边形，该截面的最小、最大横坐标 \(y\) 是多少？, 是沿横轴切，使用的是车身坐标系！
    // low:最右侧的 y,high：最左侧的 y
    // 垂直于车身 x 轴、平行于车身 y 轴的
    auto const [low, high] = lateral_extent(footprint, body_x);

    // 从障碍距离中扣除车身，得到左右净空
    // 左右障碍物的距离 - 车身的宽度 - 安全余量
    double const left_clearance =
        result.boundary.left_width[i] - high - config.safety_margin;
    double const right_clearance =
        result.boundary.right_width[i] + low - config.safety_margin;

    result.left_clearance.push_back(left_clearance);
    result.right_clearance.push_back(right_clearance);


    // 左右斜率：净空沿车身纵向如何变化
    //         实际上在计算左右净空关于纵向坐标的线性拟合斜率。
    // 这里做的是一个最小二乘法拟合
    // 中心化后的纵向坐标（Centered Longitudinal Coordinate）。
    double const centered_s = s - mean_s;
    cov_left += centered_s * left_clearance;
    cov_right += centered_s * right_clearance;
    variance += centered_s * centered_s;
    // 也就是只选择预计运动完成后，车身占据的纵向范围
    // 这也说明该曲率拟合主要针对运动结束时的车身位置，不是逐时刻的轨迹优化
    // 拟合运动结束时的车身侧移
    // 只用运动后车身所在区间拟合曲率
    // travel： 旅行，行程、（有符号行驶距离），这个 travel 表示的是后退的距离，不是自己一开始理解的向后延长的边界
    //
    if(s >= xmin + travel && s <= xmax + travel)
    {
      // 小角度下车身截面的侧移约 k*(travel^2/2 + travel*body_x)
      // 包含车头外摆与车尾侧移，且使用前后各截面的边界变化
      // desired： 这个截面希望向哪边移动多少，尽可能的向净空多的一侧移动
      // desired	期望值、目标值	希望当前车身截面横向移动多少
      // desired：期望侧移量，对于一段位移，期望它的侧移量，不要过大
      double const desired_lateral_shift =
          std::clamp(0.5 * (left_clearance - right_clearance), -0.25, 0.25);
      // coefficient	系数	描述曲率 \(k\) 对截面侧移的影响
      // 曲率对应的侧移系数  这个变量来自机器人常曲率运动模型的小角度近似。
      // 它的物理意义是：单位曲率变化对该车身截面横向位移的影响系数。
      double const lateral_shift_coefficient =
          0.5 * travel * travel + travel * body_x;
      // weight	权重	当前截面的侧移目标有多重要
      /*
          weight 表示该采样截面在优化中的重要程度。
          这里的设计原则是：
          哪一侧离障碍物越近，该截面在曲率拟合中就越重要。
      */
      double const clearance_weight =
          1.0 /
          (0.05 + std::max(0.0, std::min(left_clearance, right_clearance)));
      numerator +=
          clearance_weight * lateral_shift_coefficient * desired_lateral_shift;
      denominator += clearance_weight * lateral_shift_coefficient *
                     lateral_shift_coefficient;
    }
  }
  result.left_slope = cov_left / variance;
  result.right_slope = cov_right / variance;
  result.preferred_curvature =
      std::clamp(denominator > epsilon ? numerator / denominator : 0.0,
                 -config.max_curvature,
                 config.max_curvature);
  return result;
}

Result
plan(algorithms::GridMap<std::uint8_t> const& grid,
     Pose const& start,
     std::span<Eigen::Vector2d> footprint,
     Config const& config)
{
  Result result;
  // 这是初始位姿下，面向后退动作的左右边界信息。它主要用于解释环境和引导候选，不能单独证明轨迹安全。
  result.boundary = compute_boundary_profile(grid,
                                             start,
                                             footprint,
                                             config,
                                             -config.reverse_length);

  return {};
}

} // namespace algorithms