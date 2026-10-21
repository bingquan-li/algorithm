#pragma once

#include <Eigen/Core>
#include <atomic>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace rerun
{
class RecordingStream;
} // namespace rerun

namespace algorithms
{
namespace viz
{
class Recording;
} // namespace viz
// Shared by all libraries through algorithms::visualization; no SDK types in log calls.
class Debug
{
public:
  struct Context
  {
    std::int64_t frame_id;
    // Unix epoch nanoseconds, not a floating-point timestamp.
    std::int64_t timestamp_ns;
    // Geometric/iteration index within a frame; not execution time.
    std::int64_t step = 0;
  };
  struct Color
  {
    std::uint8_t r{};
    std::uint8_t g{};
    std::uint8_t b{};
    std::uint8_t a = 255;

    // Returns a copy; alpha uses [0, 255], where 0 is transparent.
    [[nodiscard]] constexpr Color
    with_alpha(std::uint8_t alpha) const noexcept
    {
      return {r, g, b, alpha};
    }
  };
  // Shared opaque palette, independent of the Rerun SDK.
  struct Colors
  {
    static constexpr Color black{0, 0, 0, 255};
    static constexpr Color white{245, 255, 255, 255};
    static constexpr Color gray{148, 163, 184, 255};
    static constexpr Color red{239, 68, 68, 255};
    static constexpr Color green{50, 220, 80, 255};
    static constexpr Color blue{60, 150, 255, 255};
    static constexpr Color yellow{245, 180, 45, 255};
    static constexpr Color orange{249, 115, 22, 255};
    static constexpr Color cyan{34, 211, 238, 255};
    static constexpr Color purple{140, 100, 220, 255};
    static constexpr Color pink{192, 38, 211, 255};
    static constexpr Color teal{13, 148, 136, 255};

    // Planning conventions; preserve the existing narrow-passage palette.
    static constexpr Color selected = green;
    static constexpr Color feasible = blue;
    static constexpr Color pending = yellow;
    static constexpr Color rejected{240, 80, 80, 255};
    static constexpr Color collision{255, 40, 40, 255};
    static constexpr Color footprint = teal;
    static constexpr Color obstacle{80, 175, 170, 255};
    static constexpr Color reference = gray;
  };

  struct Radius
  {
    float value;
    bool screen = false;
    constexpr Radius(float meters)
      : value(meters)
    {}
    static constexpr Radius
    ui_points(float points)
    {
      Radius result(points);
      result.screen = true;
      return result;
    }
  };

  static void
  flush();
  static void
  set_enabled(bool enabled) noexcept;
  // False when unbound, disabled, or this thread has no context.
  [[nodiscard]] static bool
  active() noexcept;
  // Apply SDK time immediately on this thread, even while logging is disabled.
  static void
  set_context(Context context) noexcept;
  // Also clears SDK time on this thread.
  static void
  clear_context() noexcept;
  [[nodiscard]] static std::optional<Context>
  context() noexcept;

  class ScopedContext
  {
  public:
    explicit ScopedContext(Context context) noexcept;
    ~ScopedContext();
    ScopedContext(ScopedContext const&) = delete;
    ScopedContext&
    operator=(ScopedContext const&) = delete;

  private:
    std::optional<Context> previous_;
  };
  // No-op before conversion/allocation when inactive. Errors propagate to the caller.
  // Radius uses scene meters, or explicit Radius::ui_points / radius_ui.
  static void
  line(std::string_view entity,
       std::span<Eigen::Vector2d const> points,
       Color color,
       Radius radius = 0.008F,
       float z = 0.02F,
       bool closed = false);
  static void
  line_xy(std::string_view entity,
          std::span<Eigen::Vector3d const> points,
          Color color,
          float radius = 0.008F,
          float z = 0.0F);
  static void
  line_2d(std::string_view entity,
          std::span<Eigen::Vector2d const> points,
          Color color,
          float radius_ui = 1.5F,
          float draw_order = 10.0F);
  static void
  point(std::string_view entity,
        Eigen::Vector2d const& point,
        Color color,
        Radius radius = 0.012F,
        float z = 0.04F,
        std::string_view label = {},
        bool show_label = false);
  static void
  arrow(std::string_view entity,
        Eigen::Vector2d const& origin,
        Eigen::Vector2d const& vector,
        Color color,
        float radius = 0.004F,
        float z = 0.04F,
        std::string_view label = {},
        bool show_label = false);
  static void
  boxes(std::string_view entity,
        std::span<Eigen::Vector3d const> centers,
        Eigen::Vector3d const& half_size,
        Color color);
  static void
  scalar(std::string_view entity, double value);
  static void
  text(std::string_view entity, std::string_view message, bool error = false);
  static void
  document(std::string_view entity, std::string_view message);
  static void
  clear(std::string_view entity, bool recursive = true);

private:
  friend class viz::Recording;
  static void
  initialize(rerun::RecordingStream const& recording);
  static void
  shutdown() noexcept;
  static void
  sync_time() noexcept;
  // Logging checks availability only; time is managed by context transitions.
  static rerun::RecordingStream const*
  active_recording() noexcept;
  // Defined in one shared library, not separate header instances in each DSO.
  static std::atomic<rerun::RecordingStream const*> recording_;
  static std::atomic<bool> enabled_;
  static thread_local std::optional<Context> context_;
};
} // namespace algorithms
