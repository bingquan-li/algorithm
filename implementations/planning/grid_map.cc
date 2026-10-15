#include "grid_map.hh"
#include <cmath>
#include <stdexcept>

namespace algorithms
{
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
GridMap<T>::at(std::size_t x, std::size_t y)
{
  if(x >= geometry_.width || y >= geometry_.height)
  {
    throw std::invalid_argument("x,y must <= geometry");
  }

  return data_[y * geometry_.width + x];
}

} // namespace algorithms