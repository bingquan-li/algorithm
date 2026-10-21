#include "debug.hh"
#include "geometry.hh"

#include <stdexcept>
#include <string>

namespace algorithms
{
std::atomic<rerun::RecordingStream const*> Debug::recording_{nullptr};
std::atomic<bool> Debug::enabled_{true};
thread_local std::optional<Debug::Context> Debug::context_;

void
Debug::initialize(rerun::RecordingStream const& recording)
{
  rerun::RecordingStream const* expected = nullptr;
  if(!recording_.compare_exchange_strong(expected, &recording))
  {
    throw std::logic_error("A Recording is already active; join workers and "
                           "destroy it before creating another");
  }
  enabled_.store(true);
  clear_context();
}

void
Debug::shutdown() noexcept
{
  clear_context();
  recording_.store(nullptr);
}


void
Debug::flush()
{
  if(auto const* rec = recording_.load())
  {
    rec->flush_blocking().throw_on_failure();
  }
}


void
Debug::set_enabled(bool enabled) noexcept
{
  enabled_.store(enabled);
}


bool
Debug::active() noexcept
{
  return active_recording() != nullptr;
}


void
Debug::set_context(Context context) noexcept
{
  context_ = context;
  sync_time();
}


void
Debug::clear_context() noexcept
{
  context_.reset();
  sync_time();
}


std::optional<Debug::Context>
Debug::context() noexcept
{
  return context_;
}


Debug::ScopedContext::ScopedContext(Context context) noexcept
  : previous_(context_)
{
  set_context(context);
}


Debug::ScopedContext::~ScopedContext()
{
  if(previous_)
  {
    set_context(*previous_);
  }
  else
  {
    clear_context();
  }
}


void
Debug::sync_time() noexcept
{
  auto const* rec = recording_.load(std::memory_order_acquire);
  if(rec == nullptr)
  {
    return;
  }
  // Apply at context transitions, including scope restoration and clearing.
  // Do not gate on enabled_: another thread may re-enable logging afterwards.
  rec->reset_time();
  if(!context_)
  {
    return;
  }
  rec->set_time_sequence("frame", context_->frame_id);
  rec->set_time_timestamp_nanos_since_epoch("timestamp",
                                            context_->timestamp_ns);
  rec->set_time_sequence("step", context_->step);
}

rerun::RecordingStream const*
Debug::active_recording() noexcept
{
  if(!enabled_.load(std::memory_order_relaxed) || !context_)
  {
    return nullptr;
  }
  return recording_.load(std::memory_order_acquire);
}
namespace
{

rerun::Color
color(Debug::Color value)
{
  return {value.r, value.g, value.b, value.a};
}

rerun::Radius
radius(Debug::Radius value)
{
  return value.screen ? rerun::Radius::ui_points(value.value)
                      : rerun::Radius(value.value);
}

} // namespace


void
Debug::line(std::string_view entity,
            std::span<Eigen::Vector2d const> points,
            Color tint,
            Radius width,
            float z,
            bool closed)
{
  auto const* rec = active_recording();
  if(rec == nullptr)
  {
    return;
  }
  std::vector<Eigen::Vector2d> polygon;
  if(closed && !points.empty() && points.front() != points.back())
  {
    polygon.assign(points.begin(), points.end());
    polygon.push_back(polygon.front());
    points = polygon;
  }
  rec->try_log(entity,
               rerun::LineStrips3D(viz::line_xy(points, z))
                   .with_colors({color(tint)})
                   .with_radii({radius(width)}))
      .throw_on_failure();
}


void
Debug::line_xy(std::string_view entity,
               std::span<Eigen::Vector3d const> points,
               Color tint,
               float radius,
               float z)
{
  if(auto const* rec = active_recording())
  {
    rec->try_log(entity,
                 rerun::LineStrips3D(viz::line_xy(points, z))
                     .with_colors({color(tint)})
                     .with_radii({radius}))
        .throw_on_failure();
  }
}


void
Debug::line_2d(std::string_view entity,
               std::span<Eigen::Vector2d const> points,
               Color tint,
               float radius_ui,
               float draw_order)
{
  auto const* rec = active_recording();
  if(rec == nullptr)
  {
    return;
  }
  std::vector<rerun::Vec2D> vertices;
  vertices.reserve(points.size());
  for(auto const& p : points)
  {
    vertices.emplace_back(float(p.x()), float(p.y()));
  }
  rec->try_log(entity,
               rerun::LineStrips2D(rerun::LineStrip2D(std::move(vertices)))
                   .with_colors({color(tint)})
                   .with_radii({rerun::Radius::ui_points(radius_ui)})
                   .with_draw_order(draw_order))
      .throw_on_failure();
}


void
Debug::point(std::string_view entity,
             Eigen::Vector2d const& p,
             Color tint,
             Radius width,
             float z,
             std::string_view label,
             bool show_label)
{
  if(auto const* rec = active_recording())
  {
    rec->try_log(entity,
                 rerun::Points3D({viz::position(p, z)})
                     .with_colors({color(tint)})
                     .with_radii({radius(width)})
                     .with_labels({std::string(label)})
                     .with_show_labels(show_label))
        .throw_on_failure();
  }
}


void
Debug::arrow(std::string_view entity,
             Eigen::Vector2d const& origin,
             Eigen::Vector2d const& vector,
             Color tint,
             float radius,
             float z,
             std::string_view label,
             bool show_label)
{
  if(auto const* rec = active_recording())
  {
    rec->try_log(entity,
                 rerun::Arrows3D::from_vectors(
                     {{float(vector.x()), float(vector.y()), 0.0F}})
                     .with_origins({viz::position(origin, z)})
                     .with_colors({color(tint)})
                     .with_radii({radius})
                     .with_labels({std::string(label)})
                     .with_show_labels(show_label))
        .throw_on_failure();
  }
}


void
Debug::boxes(std::string_view entity,
             std::span<Eigen::Vector3d const> centers,
             Eigen::Vector3d const& half_size,
             Color tint)
{
  auto const* rec = active_recording();
  if(rec == nullptr)
  {
    return;
  }
  std::vector<rerun::components::Translation3D> converted;
  converted.reserve(centers.size());
  for(auto const& p : centers)
  {
    converted.emplace_back(float(p.x()), float(p.y()), float(p.z()));
  }
  rec->try_log(
         entity,
         rerun::Boxes3D::from_centers_and_half_sizes(converted,
                                                     {{float(half_size.x()),
                                                       float(half_size.y()),
                                                       float(half_size.z())}})
             .with_colors({color(tint)})
             .with_fill_mode(rerun::FillMode::Solid))
      .throw_on_failure();
}


void
Debug::scalar(std::string_view entity, double value)
{
  if(auto const* rec = active_recording())
  {
    rec->try_log(entity, rerun::Scalars(value)).throw_on_failure();
  }
}


void
Debug::text(std::string_view entity, std::string_view message, bool error)
{
  if(auto const* rec = active_recording())
  {
    rec->try_log(entity,
                 rerun::TextLog(std::string(message))
                     .with_level(error ? rerun::TextLogLevel::Error
                                       : rerun::TextLogLevel::Debug))
        .throw_on_failure();
  }
}


void
Debug::clear(std::string_view entity, bool recursive)
{
  if(auto const* rec = active_recording())
  {
    rec->try_log(entity, rerun::Clear(recursive)).throw_on_failure();
  }
}


void
Debug::document(std::string_view entity, std::string_view message)
{
  if(auto const* rec = active_recording())
  {
    rec->try_log(entity, rerun::TextDocument(std::string(message)))
        .throw_on_failure();
  }
}
} // namespace algorithms
