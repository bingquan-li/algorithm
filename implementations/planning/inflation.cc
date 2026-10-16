#include "inflation.hh"

namespace algorithms
{

namespace
{

struct Offset
{
  int dx;
  int dy;
};

// 预计算膨胀模板
std::vector<Offset>
buildOffsets(int max_dx, int max_dy, double radius_cells, InflationShape shape)
{
  std::vector<Offset> offsets;

  double const radius_sq = radius_cells * radius_cells;

  for(int dy = -max_dy; dy <= max_dy; ++dy)
  {
    for(int dx = -max_dx; dx <= max_dx; ++dx)
    {
      // 圆形膨胀：判断格子中心是否在圆内
      if(shape == InflationShape::Circle)
      {
        double const dist_sq =
            static_cast<double>(dx) * dx + static_cast<double>(dy) * dy;

        if(dist_sq > radius_sq)
        {
          continue;
        }
      }

      offsets.push_back({dx, dy});
    }
  }

  return offsets;
}

} // namespace

GridMap<uint8_t>
inflate(GridMap<uint8_t> const& input, InflationOptions const& options)
{
  // 1. 检查参数
  if(!std::isfinite(options.radius) || options.radius < 0.0)
  {
    throw std::invalid_argument("Invalid inflation radius");
  }

  double const resolution = input.geometry().resolution;

  if(!std::isfinite(resolution) || resolution <= 0.0)
  {
    throw std::invalid_argument("Invalid map resolution");
  }

  // 2. 复制原始地图
  GridMap<uint8_t> output = input;

  int const width = input.geometry().width;
  int const height = input.geometry().height;

  if(width <= 0 || height <= 0)
  {
    return output;
  }

  // 3. 将米转换成栅格单位
  double const radius_cells = options.radius / resolution;

  // 限制搜索范围为地图大小，避免无意义的超大模板
  int const max_dx =
      static_cast<int>(std::min(std::ceil(radius_cells), double(width - 1)));

  int const max_dy =
      static_cast<int>(std::min(std::ceil(radius_cells), double(height - 1)));

  // 4. 预计算结构元素
  auto const offsets =
      buildOffsets(max_dx, max_dy, radius_cells, options.shape);

  // 5. 遍历原始地图的障碍物
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      if(input.at(x, y) != options.obstacle_value)
      {
        continue;
      }

      // 6. 将障碍物周围的栅格标记为膨胀区域
      for(auto const& offset : offsets)
      {
        // 使用更宽的整数类型，避免坐标加法溢出
        auto const nx = static_cast<long long>(x) + offset.dx;
        auto const ny = static_cast<long long>(y) + offset.dy;

        if(nx < 0 || ny < 0 || nx >= width || ny >= height)
        {
          continue;
        }

        output.at(static_cast<int>(nx), static_cast<int>(ny)) =
            options.inflated_value;
      }
    }
  }

  return output;
}

} // namespace algorithms