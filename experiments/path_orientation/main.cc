#include "straight_prefix.hh"
#include "rerun/rerun_wrapper.hh"

#include <iostream>

int
main()
{
  auto path = load_path("/workspace/algorithms/data/path.csv");
  auto result = extract(path);
  std::cout << "dir: " << result.direction;
  rerun::RecordingStream rec("path_orientation");
  rec.spawn().exit_on_failure();
  rec.log("/origin_path",
          rerun::LineStrips3D{rerun::LineStrip3D{path}}.with_radii(0.05F));
  rec.log("/straight_path",
          rerun::LineStrips3D{rerun::LineStrip3D{std::vector<Eigen::Vector2d>(
                                  path.begin(),
                                  path.begin() + result.end_index)}}
              .with_radii(0.05F));
  rec.log(
      "/direction",
      rerun::Arrows3D::from_vectors(
          {rerun::Vector3D{static_cast<float>(result.direction.x()),
                           static_cast<float>(result.direction.y()),
                           0.0}})
          .with_radii(0.008F)
          .with_origins({rerun::Position3D{static_cast<float>(path.front().x()),
                                           static_cast<float>(path.front().y()),
                                           0.0}}));
  return 0;
}