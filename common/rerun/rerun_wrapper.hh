#include <rerun.hpp>
#include <Eigen/Eigen>

namespace rerun
{
template <typename Allocator>
struct CollectionAdapter<encodings::Vec3D,
                         std::vector<Eigen::Vector2d, Allocator>>
{
  Collection<encodings::Vec3D>
  operator()(std::vector<Eigen::Vector2d, Allocator> const& points) const
  {
    std::vector<encodings::Vec3D> converted;
    converted.reserve(points.size());
    for(auto const& point : points)
    {
      converted.emplace_back(static_cast<float>(point.x()),
                             static_cast<float>(point.y()),
                             0);
    }
    // Collection 持有转换结果，传入临时 Eigen 容器也不会悬空。
    return Collection<encodings::Vec3D>::take_ownership(std::move(converted));
  }
};

template <typename Allocator>
struct CollectionAdapter<encodings::Vec3D,
                         std::vector<Eigen::Vector3d, Allocator>>
{
  Collection<encodings::Vec3D>
  operator()(std::vector<Eigen::Vector2d, Allocator> const& points) const
  {
    std::vector<encodings::Vec3D> converted;
    converted.reserve(points.size());
    for(auto const& point : points)
    {
      converted.emplace_back(static_cast<float>(point.x()),
                             static_cast<float>(point.y()),
                             static_cast<float>(point.z()));
    }
    // Collection 持有转换结果，传入临时 Eigen 容器也不会悬空。
    return Collection<encodings::Vec3D>::take_ownership(std::move(converted));
  }
};
} // namespace rerun
