#include "straight_prefix.hh"
#include "reference_line.h"
#include "rerun/rerun_wrapper.hh"

#include "grid_map.hh"

#include <iostream>

int
main()
{
  auto path = load_path(
      "/workspace/algorithms/data/narrow_passage_reverseing2/path.csv");
  auto result = extract(path);

  std::vector<Eigen::Vector2d> direction_path{path.begin(),
                                              path.begin() + result.end_index};

  // reference_line::log_xy_path("/origin_path", path);
  // reference_line::log_xy_path("/straight_path", direction_path);


  // rec.log(
  //     "/direction",
  //     rerun::Arrows3D::from_vectors(
  //         {rerun::Vector3D{static_cast<float>(result.direction.x()),
  //                          static_cast<float>(result.direction.y()),
  //                          0.0}})
  //         .with_radii(0.008F)
  //         .with_origins({rerun::Position3D{static_cast<float>(path.front().x()),
  //                                          static_cast<float>(path.front().y()),
  //                                          0.0}}));
  reference_line::Config config;
  reference_line::ReferenceLine ref(&config);
  if(direction_path.empty())
  {
    std::cout << "err construct direction path failed!";
    return 1;
  }

  Eigen::Vector2d vehicle_dir(0.2383018770177138, 0.9711911322751224);
  // if(std::abs(result.direction.dot(vehicle_dir)) > 0.03)
  // {
  //   result.direction = -vehicle_dir;
  // }

  std::filesystem::path grid_map_path(
      "/workspace/algorithms/data/narrow_passage_reverseing2/frame.json");

  auto grid_map = algorithms::load_grid_map_json(grid_map_path);

  ref.create_new_line(direction_path, result.direction, 0.1, 2.2);
  // ref.create_new_line(path3d);

  return 0;
}