#include "later_boundary_query.hh"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void
near(double actual, double expected)
{
  if(!std::isfinite(actual) || std::abs(actual - expected) > 1e-10)
  {
    throw std::runtime_error("Expected " + std::to_string(expected) + ", got " +
                             std::to_string(actual));
  }
}

template <typename Function>
void
reject(Function function)
{
  try
  {
    function();
  }
  catch(std::invalid_argument const&)
  {
    return;
  }
  throw std::runtime_error("Expected invalid_argument");
}
} // namespace

int
main()
{
  try
  {
    // 非零地图原点、非单位分辨率、不对称通道及非单位朝向。
    algorithms::GridMap<std::uint8_t> corridor({8, 10, 0.25, -2.0, -3.0});
    for(std::size_t x = 0; x < 8; ++x)
    {
      corridor(x, 1) = 2;
      corridor(x, 7) = 2;
    }
    std::array const samples{-0.25, 0.0, 0.25};
    Eigen::Vector2d const pose(-1.125, -2.125); // 栅格坐标 (3.5, 3.5)
    auto const result = build_lateral_boundary(corridor, pose, {5, 0}, samples);
    auto const reversed =
        build_lateral_boundary(corridor, pose, {-2, 0}, samples);
    if(result.left_width.size() != 3 || result.right_width.size() != 3)
    {
      throw std::runtime_error("Incorrect result size");
    }
    for(std::size_t i = 0; i < samples.size(); ++i)
    {
      near(result.left_width[i], 0.875);
      near(result.right_width[i], 0.375);
      near(reversed.left_width[i], 0.375);
      near(reversed.right_width[i], 0.875);
    }

    // 改变一个纵向采样点的障碍，确认输出确实逐点计算并保持输入顺序。
    corridor(4, 5) = 2;
    auto const varied = build_lateral_boundary(corridor, pose, {1, 0}, samples);
    near(varied.left_width[0], 0.875);
    near(varied.left_width[1], 0.875);
    near(varied.left_width[2], 0.375);

    std::array const origin_sample{0.0};
    algorithms::GridMap<std::uint8_t> grid({8, 8, 1.0, 0, 0});
    // 朝北时左侧是西，右侧是东；无障碍时返回地图边缘。
    auto widths =
        build_lateral_boundary(grid, {2.5, 3.5}, {0, 4}, origin_sample);
    near(widths.left_width[0], 2.5);
    near(widths.right_width[0], 5.5);
    grid(3, 4) = 2;
    // 左法线为 (0.6, 0.8)，进入单个障碍格 (3,4) 的距离为 0.5/0.6。
    widths = build_lateral_boundary(grid, {2.5, 3.5}, {4, -3}, origin_sample);
    near(widths.left_width[0], 5.0 / 6.0);
    near(widths.right_width[0], 25.0 / 6.0);
    // 角点相切：不能漏掉只在 (3,4) 角点接触的侧格 (3,3)。
    grid(3, 4) = 0;
    grid(3, 3) = 2;
    widths = build_lateral_boundary(grid, {2.5, 3.5}, {1, -1}, origin_sample);
    near(widths.left_width[0], std::sqrt(0.5));

    grid(3, 3) = 1;
    widths =
        build_lateral_boundary(grid, {2.5, 3.5}, {0, -1}, origin_sample, 2);
    near(widths.left_width[0], 5.5);
    widths = build_lateral_boundary(grid, {2.5, 3.5}, {0, -1}, origin_sample);
    near(widths.left_width[0], 0.5);
    widths = build_lateral_boundary(grid, {3, 3.5}, {0, 1}, origin_sample);
    near(widths.left_width[0], 0);
    near(widths.right_width[0], 0);
    // 负向射线恰好从栅格边缘进入障碍格，距离为零。
    widths = build_lateral_boundary(grid, {4, 3.5}, {0, 1}, origin_sample);
    near(widths.left_width[0], 0);
    near(widths.right_width[0], 4);

    for(Eigen::Vector2d const& outside : {Eigen::Vector2d(-0.1, 2),
                                          Eigen::Vector2d(8, 2),
                                          Eigen::Vector2d(2, 8)})
    {
      widths = build_lateral_boundary(grid, outside, {1, 0}, origin_sample);
      near(widths.left_width[0], 0);
      near(widths.right_width[0], 0);
    }
    widths = build_lateral_boundary(grid, {0, 2}, {0, 1}, origin_sample);
    near(widths.left_width[0], 0);
    near(widths.right_width[0], 8);
    auto const empty = build_lateral_boundary(grid, {2, 2}, {1, 0}, {});
    if(!empty.left_width.empty() || !empty.right_width.empty())
    {
      throw std::runtime_error("Empty samples must produce empty widths");
    }

    double const nan = std::numeric_limits<double>::quiet_NaN();
    double const inf = std::numeric_limits<double>::infinity();
    reject(
        [&]
        { (void)build_lateral_boundary(grid, {2, 2}, {0, 0}, origin_sample); });
    reject(
        [&]
        {
          (void)build_lateral_boundary(grid, {nan, 2}, {1, 0}, origin_sample);
        });
    reject(
        [&]
        {
          (void)build_lateral_boundary(grid, {2, 2}, {inf, 0}, origin_sample);
        });
    reject(
        [&]
        {
          (void)build_lateral_boundary(grid, {2, 2}, {1, 0}, origin_sample, 0);
        });
    std::array const invalid_samples{inf};
    reject(
        [&]
        {
          (void)build_lateral_boundary(grid, {2, 2}, {1, 0}, invalid_samples);
        });
    std::cout << "Lateral boundary tests passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
