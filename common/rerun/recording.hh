#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string_view>

namespace rerun
{
class RecordingStream;
} // namespace rerun

namespace algorithms::viz
{
struct RecordingOptions
{
  // File output replaces viewer spawning; useful for headless runs and tests.
  std::optional<std::filesystem::path> output;
  std::optional<std::filesystem::path> blueprint;
  // With no output file, false disables recording without spawning a viewer.
  bool viewer = true;
};

// Owns the stream, starts its sink and binds Debug in one operation.
// Create before starting workers and destroy after joining them.
// Destruction detaches Debug before destroying/flushing the stream.
class Recording
{
public:
  explicit Recording(std::string_view application_id,
                     RecordingOptions const& options = {});
  ~Recording();
  Recording(Recording const&) = delete;
  Recording&
  operator=(Recording const&) = delete;
  Recording(Recording&&) = delete;
  Recording&
  operator=(Recording&&) = delete;

private:
  std::unique_ptr<rerun::RecordingStream> stream_;
};
} // namespace algorithms::viz
