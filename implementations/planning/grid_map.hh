#pragma once

#include <cstddef>
#include <span>
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
} // namespace algorithms