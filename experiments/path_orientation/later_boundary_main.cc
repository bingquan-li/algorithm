#include "later_boundary_query.hh"
#include "local_escape_planner.hh"

#include <rerun.hpp>
#include <nlohmann/json.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
using Positions = std::vector<rerun::Position3D>;

rerun::Position3D
position(Eigen::Vector2d const& point, float z = 0.02F)
{
  return {static_cast<float>(point.x()), static_cast<float>(point.y()), z};
}

std::vector<Eigen::Vector2d>
load_footprint(std::filesystem::path const& path)
{
  std::ifstream input(path);
  auto const document = nlohmann::json::parse(input);
  auto const& footprint = document.at("foot_print");
  if(footprint.value("coordinate_system", std::string("world")) != "world" ||
     footprint.value("unit", std::string("m")) != "m")
  {
    throw std::invalid_argument(
        "Footprint must use world coordinates in meters");
  }
  auto const& data = footprint.at("data");
  if(!data.is_array() || data.size() < 3)
  {
    throw std::invalid_argument(
        "Footprint must contain at least three vertices");
  }
  std::vector<Eigen::Vector2d> points;
  for(auto const& vertex : data)
  {
    if(!vertex.is_array() || vertex.size() != 2)
    {
      throw std::invalid_argument("Footprint vertices must contain [x, y]");
    }
    Eigen::Vector2d const point(vertex.at(0).get<double>(),
                                vertex.at(1).get<double>());
    if(!point.allFinite())
    {
      throw std::invalid_argument("Footprint vertices must be finite");
    }
    points.push_back(point);
  }
  if(points.front() != points.back())
  {
    points.push_back(points.front());
  }
  return points;
}

void
log_line(rerun::RecordingStream const& rec,
         std::string const& entity,
         Positions const& points,
         rerun::Color color,
         float radius = 1.5F)
{
  std::vector<rerun::Vec3D> vertices;
  vertices.reserve(points.size());
  for(auto const& p : points)
  {
    vertices.emplace_back(p.x(), p.y(), p.z());
  }
  rec.try_log_static(
         entity,
         rerun::LineStrips3D(rerun::LineStrip3D(std::move(vertices)))
             .with_colors({color})
             .with_radii({rerun::Radius::ui_points(radius)}))
      .throw_on_failure();
}

void
log_scene(rerun::RecordingStream const& rec,
          algorithms::GridMap<std::uint8_t> const& grid,
          std::vector<Eigen::Vector2d> const& footprint,
          Eigen::Vector2d const& pose,
          Eigen::Vector2d const& dir,
          std::span<double const> samples,
          LateralBoundary const& boundary)
{
  rec.try_log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP)
      .throw_on_failure();
  auto const& geometry = grid.geometry();
  std::array<Positions, 256> cells_by_class;
  for(std::size_t y = 0; y < geometry.height; ++y)
  {
    for(std::size_t x = 0; x < geometry.width; ++x)
    {
      cells_by_class[grid(x, y)].push_back(
          position({geometry.origin_x + (x + 0.5) * geometry.resolution,
                    geometry.origin_y + (y + 0.5) * geometry.resolution},
                   -0.001F));
    }
  }
  // 每个原始格子对应一个薄方块，顶面位于 z=0；不膨胀、不重采样。
  // 颜色与 data/plot_raw_frame.py 的 class_id_0_to_3 配色一致。
  std::array const palette{rerun::Color(241, 245, 249),
                           rerun::Color(96, 165, 250),
                           rerun::Color(38, 52, 69),
                           rerun::Color(245, 158, 11)};
  auto const half_cell = static_cast<float>(geometry.resolution * 0.5);
  for(std::size_t value = 0; value < cells_by_class.size(); ++value)
  {
    if(cells_by_class[value].empty())
    {
      continue;
    }
    auto const color =
        value < palette.size() ? palette[value] : rerun::Color(120, 80, 160);
    rec.try_log_static("world/grid_map/class_" + std::to_string(value),
                       rerun::Boxes3D::from_centers_and_half_sizes(
                           cells_by_class[value],
                           {{half_cell, half_cell, 0.001F}})
                           .with_colors({color})
                           .with_fill_mode(rerun::FillMode::Solid))
        .throw_on_failure();
  }

  Positions footprint_points;
  for(auto const& p : footprint)
  {
    footprint_points.push_back(position(p, 0.01F));
  }
  log_line(rec, "world/robot/footprint", footprint_points, {225, 29, 72}, 2.5F);

  Eigen::Vector2d const forward =
      (dir / dir.cwiseAbs().maxCoeff()).normalized();
  Eigen::Vector2d const left(-forward.y(), forward.x());
  rec.try_log_static("world/robot/pose",
                     rerun::Points3D({position(pose)})
                         .with_colors({rerun::Color(225, 29, 72)})
                         .with_radii({rerun::Radius::ui_points(5.0F)}))
      .throw_on_failure();
  rec.try_log_static(
         "world/robot/heading",
         rerun::Arrows3D::from_vectors({{static_cast<float>(0.3 * forward.x()),
                                         static_cast<float>(0.3 * forward.y()),
                                         0.0F}})
             .with_origins({position(pose)})
             .with_colors({rerun::Color(225, 29, 72)})
             .with_radii({0.006F}))
      .throw_on_failure();

  Positions centers;
  Positions left_points;
  Positions right_points;
  std::vector<rerun::LineStrip3D> left_rays;
  std::vector<rerun::LineStrip3D> right_rays;
  std::vector<std::string> center_labels;
  std::vector<std::string> left_labels;
  std::vector<std::string> right_labels;
  auto const label = [](std::size_t i, char const* name, double value)
  {
    std::ostringstream text;
    text << i << ": " << name << '=' << std::fixed << std::setprecision(3)
         << value << " m";
    return text.str();
  };
  for(std::size_t i = 0; i < samples.size(); ++i)
  {
    Eigen::Vector2d const center = pose + samples[i] * forward;
    centers.push_back(position(center));
    left_points.push_back(position(center + boundary.left_width[i] * left));
    right_points.push_back(position(center - boundary.right_width[i] * left));
    auto const ray = [](rerun::Position3D const& a, rerun::Position3D const& b)
    {
      return rerun::LineStrip3D(
          std::vector<rerun::Vec3D>{{a.x(), a.y(), a.z()},
                                    {b.x(), b.y(), b.z()}});
    };
    left_rays.push_back(ray(centers.back(), left_points.back()));
    right_rays.push_back(ray(centers.back(), right_points.back()));
    center_labels.push_back(label(i, "s", samples[i]));
    left_labels.push_back(label(i, "left", boundary.left_width[i]));
    right_labels.push_back(label(i, "right", boundary.right_width[i]));
  }
  auto const log_samples = [&](std::string const& entity,
                               Positions const& points,
                               std::vector<std::string> const& labels,
                               rerun::Color color)
  {
    log_line(rec, entity + "/line", points, color);
    rec.try_log_static(entity + "/points",
                       rerun::Points3D(points)
                           .with_colors({color})
                           .with_radii({rerun::Radius::ui_points(4.0F)})
                           .with_labels(labels)
                           .with_show_labels(false))
        .throw_on_failure();
  };
  rerun::Color const yellow(245, 180, 0);
  rerun::Color const green(22, 163, 74);
  rerun::Color const blue(37, 99, 235);
  log_samples("world/centerline", centers, center_labels, yellow);
  log_samples("world/boundary/left", left_points, left_labels, green);
  log_samples("world/boundary/right", right_points, right_labels, blue);
  rec.try_log_static(
         "world/boundary/left/rays",
         rerun::LineStrips3D(left_rays).with_colors({green}).with_radii(
             {rerun::Radius::ui_points(0.75F)}))
      .throw_on_failure();
  rec.try_log_static("world/boundary/right/rays",
                     rerun::LineStrips3D(right_rays)
                         .with_colors({blue})
                         .with_radii({rerun::Radius::ui_points(0.75F)}))
      .throw_on_failure();
  rec.try_log_static(
         "legend",
         rerun::TextDocument(
             "World XY coordinates in meters; Z offsets only separate drawing "
             "layers.\n"
             "Map: original cells grouped by class ID, no inflation.\n"
             "Red: recorded footprint, query pose and heading. Yellow: sampled "
             "centerline.\n"
             "Green: left boundary. Blue: right boundary. Thin lines: measured "
             "distances.\n"
             "Point labels contain sample index and distance; enable Show "
             "labels to display.\n"
             "Widths end at the first occupied cell OR map edge; robot "
             "half-width is not subtracted.\n"
             "Boundary polylines only connect samples; they are not obstacle "
             "contours.\n"
             "Query pose/heading are the example values in "
             "later_boundary_main.cc."))
      .throw_on_failure();
}
void
log_planner(rerun::RecordingStream const& rec,
            local_escape::Result const& result,
            std::span<Eigen::Vector2d const> body_footprint)
{
  auto const polygon_positions = [&](local_escape::Pose const& pose, float z)
  {
    Positions points;
    for(auto const& p : local_escape::world_footprint(pose, body_footprint))
    {
      points.push_back(position(p, z));
    }
    points.push_back(points.front());
    return points;
  };
  for(std::size_t i = 0; i < result.candidates.size(); ++i)
  {
    auto const& candidate = result.candidates[i];
    std::string const entity = "world/planner/candidates/" + std::to_string(i);
    Positions path;
    for(auto const& pose : candidate.poses)
    {
      path.push_back(position(pose.position, 0.035F));
    }
    log_line(rec,
             entity + "/path",
             path,
             candidate.feasible ? rerun::Color(140, 100, 220, 130)
                                : rerun::Color(120, 120, 120, 70),
             0.7F);
    if(candidate.rejected_pose)
    {
      rec.try_log_static(
             entity + "/rejected_pose",
             rerun::Points3D(
                 {position(candidate.rejected_pose->position, 0.04F)})
                 .with_colors({rerun::Color(239, 68, 68)})
                 .with_radii({rerun::Radius::ui_points(2.5F)}))
          .throw_on_failure();
      log_line(rec,
               entity + "/rejected_footprint",
               polygon_positions(*candidate.rejected_pose, 0.04F),
               {239, 68, 68, 70},
               0.5F);
    }
    std::ostringstream detail;
    detail << candidate.reason
           << "\nnet reverse progress: " << candidate.reverse_progress
           << " m\nclearance lower bound: " << candidate.min_clearance
           << " m\nscore: " << candidate.score;
    rec.try_log_static("planner_details/candidate_" + std::to_string(i),
                       rerun::TextDocument(detail.str()))
        .throw_on_failure();
  }
  std::ostringstream summary;
  summary << "Status: "
          << (result.status == local_escape::Status::success ? "success"
              : result.status == local_escape::Status::start_blocked
                  ? "start_blocked"
                  : "no_path")
          << "\nCandidates: " << result.candidates.size()
          << "\nBoundary preferred curvature: "
          << result.boundary.preferred_curvature
          << " 1/m\nLeft/right clearance slopes: " << result.boundary.left_slope
          << ", " << result.boundary.right_slope << "\n";
  if(result.selected)
  {
    auto const& selected = result.candidates[*result.selected];
    summary << "Selected candidate: " << *result.selected
            << "\nNet reverse progress: " << selected.reverse_progress
            << " m\nConservative clearance: " << selected.min_clearance
            << " m\n";
    for(auto const& motion : selected.motions)
      summary << (motion.rotation != 0  ? "Rotate"
                  : motion.distance < 0 ? "Reverse"
                                        : "Forward")
              << ": distance=" << motion.distance
              << " m, curvature=" << motion.curvature
              << " 1/m, rotation=" << motion.rotation << " rad\n";
    Positions path;
    for(auto const& pose : selected.poses)
    {
      path.push_back(position(pose.position, 0.06F));
    }
    log_line(rec, "world/planner/selected/path", path, {192, 38, 211}, 3.0F);
    std::size_t const stride =
        std::max<std::size_t>(1, (selected.poses.size() + 15) / 16);
    for(std::size_t i = 0; i < selected.poses.size(); i += stride)
    {
      log_line(rec,
               "world/planner/selected/footprints/" + std::to_string(i),
               polygon_positions(selected.poses[i], 0.05F),
               {192, 38, 211, 130},
               1.0F);
    }
    log_line(rec,
             "world/planner/selected/final_footprint",
             polygon_positions(selected.poses.back(), 0.05F),
             {192, 38, 211},
             2.0F);
    // 时间轴是几何采样序号，不是带速度约束的执行时间。
    for(std::size_t i = 0; i < selected.poses.size(); ++i)
    {
      rec.set_time_sequence("trajectory_sample", static_cast<std::int64_t>(i));
      auto const points = polygon_positions(selected.poses[i], 0.07F);
      std::vector<rerun::Vec3D> vertices;
      for(auto const& p : points)
      {
        vertices.emplace_back(p.x(), p.y(), p.z());
      }
      rec.try_log("world/planner/selected/animated_footprint",
                  rerun::LineStrips3D(rerun::LineStrip3D(std::move(vertices)))
                      .with_colors({rerun::Color(16, 185, 129)})
                      .with_radii({rerun::Radius::ui_points(2.5F)}))
          .throw_on_failure();
    }
  }
  summary
      << "\nPurple: selected path and footprints. Green: animated footprint.\n"
         "Gray: rejected or reposition-only prefixes. Violet: feasible "
         "candidates.\n"
         "Red: first rejected pose/footprint (collision OR insufficient safety "
         "margin).\n"
         "No selection means stop; candidates are diagnostics, not executable "
         "paths.\n"
         "Search is bounded; no_path does not prove that no escape exists.\n"
         "Static geometric plan only; trajectory_sample is not execution time.";
  rec.try_log_static("planner_summary", rerun::TextDocument(summary.str()))
      .throw_on_failure();
  std::cerr << summary.str() << '\n';
}
} // namespace

int
main(int argc, char** argv)
{
  try
  {
    std::filesystem::path const grid_map_path =
        argc > 1 ? argv[1]
                 : "/workspace/algorithms/data/narrow_passage_reverseing2/"
                   "frame.json";
    auto const grid_map = algorithms::load_grid_map_json(grid_map_path);
    auto const footprint = load_footprint(grid_map_path);
    Eigen::Vector2d const vehicle_pose(-2.519150772556948,
                                       -0.48583814710976636);
    Eigen::Vector2d const vehicle_dir(0.2383018770177138, 0.9711911322751224);

    // 示例：车后 0.5 m 至车前 0.5 m，每 0.1 m 查询一个中心线点。
    std::vector<double> longitudinal_samples;
    for(int i = -5; i <= 5; ++i)
    {
      longitudinal_samples.push_back(i * 0.1);
    }
    auto const boundary = build_lateral_boundary(grid_map,
                                                 vehicle_pose,
                                                 vehicle_dir,
                                                 longitudinal_samples);
    auto const& [left_width, right_width] = boundary;
    std::cout << "s_m,left_width_m,right_width_m\n";
    for(std::size_t i = 0; i < longitudinal_samples.size(); ++i)
    {
      std::cout << longitudinal_samples[i] << ',' << left_width[i] << ','
                << right_width[i] << '\n';
    }


    Eigen::Vector2d const forward = vehicle_dir.normalized();
    Eigen::Vector2d const left(-forward.y(), forward.x());
    std::vector<Eigen::Vector2d> body_footprint;
    for(auto const& point : footprint)
    {
      Eigen::Vector2d const offset = point - vehicle_pose;
      body_footprint.emplace_back(offset.dot(forward), offset.dot(left));
    }
    if(body_footprint.front() == body_footprint.back())
    {
      body_footprint.pop_back();
    }

    local_escape::Config planner_config;
    // vehicle_pose 应对应差速底盘的旋转中心；配置单位：m、rad、1/m。
    local_escape::Pose const start{vehicle_pose,
                                   std::atan2(forward.y(), forward.x())};
    auto const plan =
        local_escape::plan(grid_map, start, body_footprint, planner_config);

    rerun::RecordingStream rec("lateral_boundary");
    rec.spawn().throw_on_failure();
    log_scene(rec,
              grid_map,
              footprint,
              vehicle_pose,
              vehicle_dir,
              longitudinal_samples,
              boundary);
    log_planner(rec, plan, body_footprint);
    rec.flush_blocking().throw_on_failure();
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
