#include "debug.h"
#include "rerun/debug.hh"
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <type_traits>

namespace
{
// JSONL 按事件追加；每次运行先写 session，sequence 在 session 内递增。
// 文件失败显式抛出，不能悄悄丢失用户要求的记录。
class JsonlSink
{
public:
  bool
  enabled() const
  {
    return file_.is_open();
  }
  JsonlSink()
  {
    auto const* path = std::getenv("REFERENCE_LINE_DEBUG_FILE");
    if(!path || !*path)
      return;
    file_.open(path, std::ios::app);
    if(!file_)
    {
      throw std::runtime_error("Cannot open debug JSONL: " + std::string(path));
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
    if(auto const context = algorithms::Debug::context())
    {
      event["timeline"] = {{"name", "frame"}, {"value", context->frame_id}};
      event["frame_timestamp_unix_ns"] = context->timestamp_ns;
    }
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
algorithms::Debug::Color
rerun_color(Color const& color)
{
  return {color[0], color[1], color[2], color[3]};
}

template <typename Points>
void
xy_line(std::string const& entity,
        Points const& points,
        Color color,
        float radius)
{
  if(!algorithms::Debug::active() && !file_sink().enabled())
    return;
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
  if constexpr(std::is_same_v<typename Points::value_type, Eigen::Vector2d>)
  {
    algorithms::Debug::line(entity, points, rerun_color(color), radius, 0.0F);
  }
  else
  {
    algorithms::Debug::line_xy(entity, points, rerun_color(color), radius);
  }
}

void
sl_line(char const* entity, std::vector<reference_line::SLPoint> const& points)
{
  if(!algorithms::Debug::active() && !file_sink().enabled())
    return;
  auto data = nlohmann::json::array();
  for(auto const& p : points)
  {
    data.push_back({p.s, p.l});
  }
  file_sink().write({{"type", "line_strip_2d"},
                     {"entity", entity},
                     {"coordinate_system", "sl"},
                     {"points", std::move(data)},
                     {"color_rgba", {255, 211, 65, 255}},
                     {"radius", 1.5},
                     {"radius_unit", "ui_points"},
                     {"draw_order", 10.0}});
  if(algorithms::Debug::active())
  {
    std::vector<Eigen::Vector2d> vertices;
    vertices.reserve(points.size());
    for(auto const& p : points)
      vertices.emplace_back(p.s, p.l);
    algorithms::Debug::line_2d(entity, vertices, {255, 211, 65, 255});
  }
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
  algorithms::Debug::text("/logs/reference_line", message);
}
void
error_log(std::string const& message)
{
  file_sink().write({{"type", "text_log"},
                     {"entity", "/logs/reference_line"},
                     {"level", "Error"},
                     {"message", message}});
  algorithms::Debug::text("/logs/reference_line", message, true);
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
