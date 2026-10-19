#include "grid_map.hh"

#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

namespace algorithms
{
namespace
{
std::size_t
read_dimension(nlohmann::json const& value, char const* name)
{
  if(!value.is_number_integer() ||
     (!value.is_number_unsigned() && value.get<std::int64_t>() <= 0))
  {
    throw std::invalid_argument(std::string("GridMap: ") + name +
                                " must be a positive integer");
  }
  auto const size = value.get<std::uint64_t>();
  if(size == 0 || size > std::numeric_limits<std::size_t>::max())
  {
    throw std::invalid_argument(std::string("GridMap: invalid ") + name);
  }
  return static_cast<std::size_t>(size);
}
} // namespace

GridMap<std::uint8_t>
load_grid_map_json(std::filesystem::path const& path)
{
  std::ifstream input(path);
  if(!input)
  {
    throw std::runtime_error("GridMap: cannot open JSON file: " + path.string());
  }

  try
  {
    auto const document = nlohmann::json::parse(input);
    auto const& map = document.at("local_map");
    GridGeometry geometry;
    geometry.width = read_dimension(map.at("width"), "width");
    geometry.height = read_dimension(map.at("height"), "height");
    geometry.resolution = map.at("map_scale").get<double>();
    auto const& origin = map.at("origin");
    if(!origin.is_array() || origin.size() != 2)
    {
      throw std::invalid_argument("GridMap: origin must contain [x, y]");
    }
    geometry.origin_x = origin.at(0).get<double>();
    geometry.origin_y = origin.at(1).get<double>();

    // GridGeometry 无旋转字段，避免加载后丢失地图的空间变换。
    if(map.contains("rotation_xyzw"))
    {
      auto const& rotation = map.at("rotation_xyzw");
      if(!rotation.is_array() || rotation.size() != 4 ||
         rotation.at(0).get<double>() != 0.0 ||
         rotation.at(1).get<double>() != 0.0 ||
         rotation.at(2).get<double>() != 0.0 ||
         std::abs(rotation.at(3).get<double>()) != 1.0)
      {
        throw std::invalid_argument("GridMap: only unrotated maps are supported");
      }
    }

    auto const row_order = map.value("row_order", std::string("bottom_to_top"));
    if(row_order != "bottom_to_top" && row_order != "top_to_bottom")
    {
      throw std::invalid_argument("GridMap: unsupported row_order: " + row_order);
    }
    auto const& rows = map.at("data");
    if(!rows.is_array() || rows.size() != geometry.height)
    {
      throw std::invalid_argument("GridMap: data row count must equal height");
    }
    for(auto const& row : rows)
    {
      if(!row.is_array() || row.size() != geometry.width)
      {
        throw std::invalid_argument("GridMap: data column count must equal width");
      }
    }

    GridMap<std::uint8_t> result(geometry);
    for(std::size_t row = 0; row < geometry.height; ++row)
    {
      auto const y = row_order == "top_to_bottom" ? geometry.height - 1 - row : row;
      for(std::size_t x = 0; x < geometry.width; ++x)
      {
        auto const& value = rows[row][x];
        if(!value.is_number_integer() ||
           (!value.is_number_unsigned() && value.get<std::int64_t>() < 0) ||
           value.get<std::uint64_t>() > 255)
        {
          throw std::invalid_argument("GridMap: data[" + std::to_string(row) +
                                      "][" + std::to_string(x) +
                                      "] must be an integer in [0, 255]");
        }
        result(x, y) = value.get<std::uint8_t>();
      }
    }
    return result;
  }
  catch(nlohmann::json::exception const& error)
  {
    throw std::invalid_argument("GridMap: invalid JSON in " + path.string() +
                                ": " + error.what());
  }
}
} // namespace algorithms
