#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>


namespace algorithms
{

struct GridGeometry
{
  std::size_t width = 0;
  std::size_t height = 0;

  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
};

template <typename T>
class GridMap
{
public:
  explicit GridMap(GridGeometry geometry, T initial_value = T{});
  [[nodiscard]] bool
  contains(int x, int y) const noexcept;
  T&
  operator()(std::size_t x, std::size_t y) noexcept
  {
    return data_[y * geometry_.width + x];
  }
  T
  operator()(std::size_t x, std::size_t y) const noexcept
  {
    return data_[y * geometry_.width + x];
  }
  T&
  at(std::size_t x, std::size_t y);
  T
  at(std::size_t x, std::size_t y) const;
  [[nodiscard]] GridGeometry const&
  geometry() const noexcept
  {
    return geometry_;
  }
  std::span<T>
  cells() noexcept
  {
    return {data_.data(), data_.size()};
  }


private:
  GridGeometry geometry_;
  std::vector<T> data_;
};

template <typename T>
GridMap<T>::GridMap(GridGeometry geometry, T initial_value)
  : geometry_(geometry)
{
  if(!std::isfinite(geometry_.resolution) || geometry_.resolution <= 0.0)
  {
    throw std::invalid_argument(
        "GridMap: resolution must be finite and positive");
  }

  if(!std::isfinite(geometry_.origin_x) || !std::isfinite(geometry_.origin_y))
  {
    throw std::invalid_argument("GridMap: origin must be finite");
  }

  auto const width = geometry_.width;
  auto const height = geometry_.height;

  // 先检查，再相乘，避免 size_t 乘法溢出。
  if(height != 0 && width > std::numeric_limits<std::size_t>::max() / height)
  {
    throw std::length_error("GridMap: width * height overflows size_t");
  }

  auto const cell_count = width * height;

  // vector 可容纳的元素数还受到元素大小等因素限制。
  if(cell_count > data_.max_size())
  {
    throw std::length_error("GridMap: cell count exceeds vector max_size");
  }

  data_.assign(cell_count, initial_value);
}


template <typename T>
[[nodiscard]] bool
GridMap<T>::contains(int x, int y) const noexcept
{
  return x >= 0 && y >= 0 && static_cast<std::size_t>(x) < geometry_.width &&
         static_cast<std::size_t>(y) < geometry_.height;
}

template <typename T>
T&
GridMap<T>::GridMap<T>::at(std::size_t x, std::size_t y)
{
  if(x >= geometry_.width || y >= geometry_.height)
  {
    throw std::invalid_argument("x,y must <= geometry");
  }

  return data_[y * geometry_.width + x];
}

template <typename T>
T
GridMap<T>::at(std::size_t x, std::size_t y) const
{
  if(x >= geometry_.width || y >= geometry_.height)
  {
    throw std::invalid_argument("x,y must <= geometry");
  }

  return data_[y * geometry_.width + x];
}

// 从包含 local_map 的 JSON 文件加载 uint8 栅格。data[y][x] 按行存储，
// 尺寸必须与 width/height 一致；map_scale 为分辨率，origin 为左下角。
// 支持 bottom_to_top（默认）及 top_to_bottom 行序；仅支持无旋转地图。
// 文件读取失败抛出 runtime_error，格式或数据非法抛出 invalid_argument。
[[nodiscard]] GridMap<std::uint8_t>
load_grid_map_json(std::filesystem::path const& path);

} // namespace algorithms
