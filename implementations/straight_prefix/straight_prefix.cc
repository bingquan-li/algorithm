#include "straight_prefix.hh"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

std::vector<Eigen::Vector2d>
load_path(std::string_view file_path)
{
  std::string const path(file_path);
  std::ifstream input(path);
  if(!input.is_open())
  {
    throw std::runtime_error("Cannot open path CSV: " + path);
  }

  auto const trim = [](std::string const& value)
  {
    auto const first = value.find_first_not_of(" \t\r\n");
    if(first == std::string::npos)
    {
      return std::string{};
    }
    auto const last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
  };

  std::vector<Eigen::Vector2d> points;
  std::string line;
  std::size_t line_number = 0;
  bool first_row = true;
  while(std::getline(input, line))
  {
    ++line_number;
    // Accept UTF-8 CSV files with a byte order mark.
    if(line_number == 1 && line.starts_with("\xEF\xBB\xBF"))
    {
      line.erase(0, 3);
    }
    if(trim(line).empty())
    {
      continue;
    }

    std::istringstream row(line);
    std::string x_field;
    std::string y_field;
    std::getline(row, x_field, ',');
    std::getline(row, y_field, ',');
    x_field = trim(x_field);
    y_field = trim(y_field);
    if(first_row && x_field == "x" && y_field == "y")
    {
      first_row = false;
      continue;
    }
    first_row = false;

    auto const parse_coordinate = [&](std::string const& field)
    {
      std::istringstream value(field);
      double coordinate = 0;
      if(!(value >> coordinate) || !(value >> std::ws).eof() ||
         !std::isfinite(coordinate))
      {
        throw std::runtime_error("Invalid x/y coordinate in " + path +
                                 " at line " + std::to_string(line_number));
      }
      return coordinate;
    };
    double const x = parse_coordinate(x_field);
    double const y = parse_coordinate(y_field);
    points.emplace_back(x, y);
  }
  if(input.bad())
  {
    throw std::runtime_error("Failed to read path CSV: " + path);
  }
  return points;
}


std::vector<LocalHeading>
local_headings(std::vector<Eigen::Vector2d> const& path,
               std::vector<double> const& s,
               double window,
               double epsilon)
{
  std::vector<LocalHeading> out(path.size());
  std::size_t j = 0;
  for(std::size_t i = 1; i < path.size(); ++i)
  {
    if(s[i] < window)
    {
      continue;
    }

    double const target = s[i] - window;
    while(j + 1 < i && s[j] <= target)
    {
      ++j;
    }
    double const segment_length = s[j + 1] - s[j];
    if(segment_length <= 0)
    {
      continue;
    }

    double const ratio = std::clamp((target - s[j]) / segment_length, 0.0, 1.0);
    Eigen::Vector2d const start = path[j] + (path[j + 1] - path[j]) * ratio;
    Eigen::Vector2d const chord = path[i] - start;
    if(double const length = chord.norm(); length > epsilon)
    {
      out[i].valid = true;
      out[i].direction = chord * (1.0 / length);
    }
  }
  return out;
}

Result
extract(std::vector<Eigen::Vector2d> const& path)
{
  std::vector<double> s(path.size(), 0);
  std::vector<Eigen::Vector2d> p(path.size());
  for(std::size_t i = 1; i < path.size(); ++i)
  {
    s[i] = s[i - 1] + (path[i] - path[i - 1]).norm();
    // 这里相减得到的不仅有相对位移, 其实也可以理解成一个向量!
    p[i] = path[i] - path.front();
  }

  auto local_heading = local_headings(p, s, 0.3, 0.05);
  Moments moments;
  Result result;
  // 每加入一个点,先更新 moments, 得到新的候选直线; 检查通过后, 才更新 result. 如果检查失败, 就直接返回旧的 result.
  for(std::size_t end = 0; end < p.size(); ++end)
  {
    moments.add(p[end]);
    // todo magic, 后面移动到配置中
    if(s[end] < 0.15)
    {
      continue;
    }

    ///// 为本次候选建立的临时变量
    // 本次拟合的单位方向
    Eigen::Vector2d dir;
    // 本次检查中最大横向偏差
    double max_distance{0.0};
    // 本次检查中的最大局部方向偏差
    double max_heading{0.0};

    bool stop{false};

    // 注意这里的, p[end] 实际表示的是从 path 的起点到当前点的向量!
    // 这里检查两个事情:1.首尾跨度是否足够(当前点到起始点的直线距离),2.点集是否有确定的方向
    if(p[end].norm() < 0.15 || !moments.direction(p[end], dir))
    {
      stop = true;
    }

    // moments.mean 表示拟合直线的质心, 结合 dir, 就获得了当前的拟合直线!
    //              拟合直线的方程: L(l) = moments.mean + l*dir
    // 将方向逆时针旋转 90 度得到的单位法向量, 用它来计算点偏离直线的程度!
    Eigen::Vector2d const n{-dir.y(), dir.x()};
    for(std::size_t i = 0; i <= end; ++i)
    {
      max_distance =
          std::max(max_distance, std::abs(n.dot(p[i] - moments.mean)));

      // 存在偏差过大的点
      if(max_distance > 0.1)
      {
        stop = true;
        break;
      }

      // 检查是否有沿拟合方向进行折返的
      // 将 [p[i-1], p[i]] 投影到 dir 上
      if(i > 0 && dir.dot(p[i] - p[i - 1]) < -0.1)
      {
        stop = true;
        break;
      }

      // 检查局部方向是否和整体方向一致
      auto const& q = local_heading[i].direction;
      // 对于二维向量: 点积:cos(theta), 叉积:sin(theta)
      double const angle =
          std::abs(std::atan2((dir.x() * q.y() - dir.y() * q.x()), dir.dot(q)));
      max_heading = std::max(max_heading, angle);

      if(max_heading > 3)
      {
        stop = true;
        break;
      }
    }
    if(stop)
    {
      result.first_rejected_index = end;
      if(!result.valid)
      {
        result.direction = dir;
        result.end_index = end;
        result.normal = {-dir.y(), dir.x()};
        result.heading = std::atan2(dir.y(), dir.x());
      }
      return result;
    }
    result.valid = true;
    result.end_index = end;
    result.point_count = end + 1;
    result.length = s[end];
    result.line_point = path.front() + moments.mean;
    result.direction = dir;
    result.normal = {-dir.y(), dir.x()};
    result.heading = std::atan2(dir.y(), dir.x());
    result.max_heading_error = max_heading;
    result.max_distance = max_distance;
  }

  return result;
}
