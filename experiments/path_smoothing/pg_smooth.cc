#include "file.hh"
#include "rerun/rerun_wrapper.hh"
#include "path_smoothing.hh"

int
main()
{
  auto path = algorithms::load_path_from_csv(
      "/workspace/algorithms/data/narrow_passage_reversing/path.csv");
  rerun::RecordingStream rec("pg_smooth");
  rec.spawn().exit_on_failure();
  rec.log("pg_origin",
          rerun::LineStrips3D({rerun::LineStrip3D{path}}).with_radii(0.005F));
  path_smoothing::ProblemOptions po;
  po.fix_endpoints = true;
  po.reference_weight = 2;
  po.smoothness_weight = 10;
  po.tolerance = 1e-6;
  path_smoothing::ProjectedGradientOptions pgo;
  pgo.max_displacement = 0.1;
  pgo.max_iterator = 1000;
  pgo.shape = path_smoothing::RegionShape::Square;
  path_smoothing::Options options;
  options.problem = po;
  options.solver = pgo;

  auto result = path_smoothing::smooth(path, options);
  rec.log("pg_smooth",
          rerun::LineStrips3D({rerun::LineStrip3D{result.points}})
              .with_radii(0.005F)
              .with_colors({rerun::Color{255, 122, 133}}));
  return 0;
}