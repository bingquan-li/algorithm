#include "debug.h"
#include <rerun.hpp>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <fstream>
#include <mutex>
#include <stdexcept>

namespace
{
// JSONL 按事件追加；每次运行先写 session，sequence 在 session 内递增。
// 文件失败显式抛出，不能悄悄丢失用户要求的记录。
class JsonlSink
{
public:
  JsonlSink()
  {
    // auto const* path = std::getenv("REFERENCE_LINE_DEBUG_FILE");
    // if(!path || !*path)
    //   return;
    file_.open(
        "/workspace/algorithms/data/narrow_passage_reverseing2/debug.json",
        std::ios::app);
    if(!file_)
    {
      throw std::runtime_error(
          "Cannot open debug JSONL: " +
          std::string("/workspace/algorithms/data/narrow_passage_reverseing2/"
                      "debug.json"));
    }
    write({{"type", "session"},
           {"application_id", "reference_line"},
           {"view_coordinates", "RIGHT_HAND_Z_UP"},
           {"length_unit", "m"}});
  }

  void
  write(nlohmann::json event)
  {
    if(!file_.is_open())
    {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    event["schema_version"] = 1;
    event["sequence"] = sequence_++;
    event["timestamp_unix_ns"] =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    event["timeline"] = {{"name", "planning_cycle"}, {"value", 0}};
    file_ << event.dump() << '\n';
    file_.flush();
    if(!file_)
    {
      throw std::runtime_error("Failed to write debug JSONL");
    }
  }

private:
  std::ofstream file_;
  std::mutex mutex_;
  std::uint64_t sequence_{0};
};

JsonlSink&
file_sink()
{
  static JsonlSink sink;
  return sink;
}

using Color = std::array<unsigned char, 4>;
rerun::Color
rerun_color(Color const& color)
{
  return {color[0], color[1], color[2], color[3]};
}

rerun::RecordingStream&
recording()
{
  static rerun::RecordingStream rec("reference_line");
  static bool const initialized = [&]
  {
    rec.spawn().exit_on_failure();
    rec.set_time_sequence("planning_cycle", 0);
    rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
    return true;
  }();
  (void)initialized;
  return rec;
}

template <typename Points>
void
xy_line(std::string const& entity,
        Points const& points,
        Color color,
        float radius)
{
  std::vector<rerun::encodings::Vec3D> vertices;
  vertices.reserve(points.size());
  for(auto const& p : points)
    vertices.emplace_back(static_cast<float>(p.x()),
                          static_cast<float>(p.y()),
                          0.0F);
  auto data = nlohmann::json::array();
  for(auto const& p : points)
  {
    data.push_back({p.x(), p.y(), 0.0});
  }
  file_sink().write({{"type", "line_strip_3d"},
                     {"entity", entity},
                     {"coordinate_system", "xy"},
                     {"points", std::move(data)},
                     {"color_rgba", color},
                     {"radius", radius},
                     {"radius_unit", "m"}});
  recording().log(entity,
                  rerun::LineStrips3D(rerun::LineStrip3D(std::move(vertices)))
                      .with_colors({rerun_color(color)})
                      .with_radii({radius}));
}

void
sl_line(char const* entity, std::vector<reference_line::SLPoint> const& points)
{
  std::vector<rerun::encodings::Vec2D> vertices;
  vertices.reserve(points.size());
  for(auto const& p : points)
    vertices.emplace_back(static_cast<float>(p.s), static_cast<float>(p.l));
  auto data = nlohmann::json::array();
  for(auto const& p : points)
    data.push_back({p.s, p.l});
  file_sink().write({{"type", "line_strip_2d"},
                     {"entity", entity},
                     {"coordinate_system", "sl"},
                     {"points", std::move(data)},
                     {"color_rgba", {255, 211, 65, 255}},
                     {"radius", 1.5},
                     {"radius_unit", "ui_points"},
                     {"draw_order", 10.0}});
  recording().log(
      entity,
      rerun::LineStrips2D(rerun::components::LineStrip2D(std::move(vertices)))
          .with_colors({rerun::Color(255, 211, 65, 255)})
          .with_radii({rerun::Radius::ui_points(1.5F)})
          .with_draw_order(10.0F));
}
} // namespace

namespace reference_line
{
void
debug_log(std::string const& message)
{
  file_sink().write({{"type", "text_log"},
                     {"entity", "/logs/reference_line"},
                     {"level", "Debug"},
                     {"message", message}});
  recording().log(
      "/logs/reference_line",
      rerun::TextLog(message).with_level(rerun::TextLogLevel::Debug));
}
void
error_log(std::string const& message)
{
  file_sink().write({{"type", "text_log"},
                     {"entity", "/logs/reference_line"},
                     {"level", "Error"},
                     {"message", message}});
  recording().log(
      "/logs/reference_line",
      rerun::TextLog(message).with_level(rerun::TextLogLevel::Error));
}
void
log_xy_global_path(std::string const& entity,
                   std::vector<Eigen::Vector3d> const& points)
{
  xy_line(entity, points, Color{245, 255, 255, 255}, 0.005F);
}
void
log_xy_path(std::string const& entity,
            std::vector<Eigen::Vector2d> const& points)
{
  xy_line(entity, points, Color{245, 255, 255, 255}, 0.005F);
}
void
log_xy_lane_boundary(std::string const& entity,
                     std::vector<Eigen::Vector3d> const& points)
{
  xy_line(entity, points, Color{255, 211, 65, 255}, 0.01F);
}
void
log_xy_center_line(std::string const& entity,
                   std::vector<Eigen::Vector3d> const& points)
{
  xy_line(entity, points, Color{255, 135, 125, 40}, 0.06F);
}
void
log_sl_left_boundary(std::vector<SLPoint> const& points)
{
  sl_line("/sl/left_boundary", points);
}
void
log_sl_right_boundary(std::vector<SLPoint> const& points)
{
  sl_line("/sl/right_boundary", points);
}
} // namespace reference_line
