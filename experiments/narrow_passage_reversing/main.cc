#include "grid_map.hh"
#include "footprint.hh"
#include "rerun/rerun_wrapper.hh"
#include "lateral_boundary.hh"


#include <rerun.hpp>



#include <vector>
#include <cstdint>
#include <iostream>
#include <string_view>


template <typename IsOccupied>
void
log_grid_obstacles(rerun::RecordingStream const& rec,
                   std::string_view entity_path,
                   algorithms::GridMap<std::uint8_t> const& map,
                   IsOccupied is_occupied,
                   float display_height = 0.3F)
{
  auto const& g = map.geometry();

  std::vector<rerun::components::Translation3D> centers;
  std::vector<rerun::components::HalfSize3D> half_sizes;

  auto const half_cell = static_cast<float>(g.resolution * 0.5);
  float const half_height = display_height * 0.5F;

  for(std::size_t y = 0; y < g.height; ++y)
  {
    for(std::size_t x = 0; x < g.width; ++x)
    {
      if(!is_occupied(map(x, y)))
      {
        continue; // 自由区域不生成任何几何体。
      }

      auto const cx = static_cast<float>(
          g.origin_x + (static_cast<double>(x) + 0.5) * g.resolution);
      auto const cy = static_cast<float>(
          g.origin_y + (static_cast<double>(y) + 0.5) * g.resolution);

      centers.emplace_back(cx, cy, half_height);
      half_sizes.emplace_back(half_cell, half_cell, half_height);
    }
  }

  rec.log(entity_path,
          rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes)
              .with_colors({rerun::Rgba32(80, 175, 170)})
              .with_fill_mode(rerun::FillMode::Solid));
}

rerun::Position3D
position(Eigen::Vector2d const& point, float z = 0.02F)
{
  return {static_cast<float>(point.x()), static_cast<float>(point.y()), z};
}

void
log_line(rerun::RecordingStream const& rec,
         std::string const& name,
         std::vector<Eigen::Vector2d> const& points,
         rerun::Color color)
{
  rec.log(name,
          rerun::LineStrips3D{rerun::LineStrip3D{points}}
              .with_colors({color})
              .with_radii({0.008F}));
}


void
log_footprint(rerun::RecordingStream const& rec,
              std::string_view entity_path,
              algorithms::Footprint const& footprint)
{
  if(footprint.vertices.size() < 3)
  {
    return;
  }

  using Point = Eigen::Vector2d;

  std::string const root(entity_path);



  Point const& center = footprint.centroid;
  Point const forward = footprint.forward.normalized();

  // 1. 八边形轮廓
  auto outline = footprint.vertices;
  outline.push_back(outline.front());

  log_line(rec, root + "outline", outline, rerun::Color(13, 148, 136));

  // 2. 车身纵向中心线
  Point const rear_end = center - footprint.rear_extent * forward;
  Point const front_end = center + footprint.front_extent * forward;

  log_line(rec,
           root + "longitudinal_axis",
           {rear_end, front_end},
           rerun::Color(148, 163, 184));

  // 3. 前进方向
  double const length = footprint.length();
  Point const arrow = forward * std::max(0.1, length * 0.45);

  rec.log(root + "/forward",
          rerun::Arrows3D::from_vectors({{static_cast<float>(arrow.x()),
                                          static_cast<float>(arrow.y()),
                                          0.0F}})
              .with_origins({rerun::Position3D{static_cast<float>(center.x()),
                                               static_cast<float>(center.y()),
                                               0.0}})
              .with_colors({rerun::Color(220, 38, 38)})
              .with_radii({0.025F})
              .with_labels({"Forward"})
              .with_show_labels(true));
}

int
main()
{
  try
  {
    std::filesystem::path const grid_map_path =
        "/workspace/algorithms/data/narrow_passage_reverseing2/"
        "frame.json";
    auto map = algorithms::load_grid_map_json(grid_map_path);
    rerun::RecordingStream rec("narrow_passage");
    rec.spawn().exit_on_failure();
    rec.log_file_from_path(
        "/workspace/algorithms/algorithms/narrow_passage.rbl");
    auto const is_occupied = [](std::uint8_t value) { return value != 0; };
    log_grid_obstacles(rec, "gridmap", map, is_occupied);

    auto const footprint = algorithms::load_footprint(grid_map_path);
    std::cout << "siz:" << footprint.vertices.size() << '\n';
    log_footprint(rec, "gridmap/footprint", footprint);

    // 示例：车后 0.5 m 至车前 0.5 m，每 0.1 m 查询一个中心线点。
    double const length = footprint.length() + 0.6;
    int count = length / 0.1;
    std::vector<double> longitudinal_samples;
    for(int i = -3; i <= count - 3; ++i)
    {
      longitudinal_samples.push_back(i * 0.1);
    }
    auto lateray_bounday = algorithms::latter_boundary::build_lateral_boundary(
        map,
        footprint.centroid,
        footprint.forward,
        longitudinal_samples);
    log_line(rec,
             "left_bounds",
             lateray_bounday.left_points,
             rerun::Color{255, 0, 0});
    log_line(rec,
             "right_bounds",
             lateray_bounday.right_points,
             rerun::Color{255, 0, 0});

             
  }
  catch(std::exception const& e)
  {
    std::cout << "error: " << e.what() << '\n';
  }
  return 0;
}