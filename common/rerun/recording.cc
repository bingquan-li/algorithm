#include "recording.hh"
#include "debug.hh"
#include <rerun.hpp>

#include <stdexcept>

namespace algorithms::viz
{
namespace
{
void
start(rerun::RecordingStream const& rec, RecordingOptions const& options)
{
  if(options.blueprint && !std::filesystem::is_regular_file(*options.blueprint))
  {
    throw std::invalid_argument("Blueprint not found: " +
                                options.blueprint->string());
  }
  if(options.output)
  {
    if(options.output->empty())
    {
      throw std::invalid_argument("Empty recording output path");
    }
    rec.save(options.output->string()).throw_on_failure();
  }
  else
  {
    auto const status = rec.spawn();
    if(status.is_err())
    {
      throw std::runtime_error(
          "Cannot start Rerun viewer: " + status.description +
          " (use --save FILE.rrd for headless recording)");
    }
  }
  rec.try_log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP)
      .throw_on_failure();
  if(options.blueprint)
  {
    rec.try_log_file_from_path(options.blueprint->string()).throw_on_failure();
  }
}
} // namespace

Recording::Recording(std::string_view application_id,
                     RecordingOptions const& options)
{
  if(!options.viewer && !options.output)
  {
    return;
  }
  stream_ = std::make_unique<rerun::RecordingStream>(application_id);
  // Claim the binding before opening a file or spawning a viewer. A duplicate
  // session must not truncate the active session's output or unbind it.
  Debug::initialize(*stream_);
  try
  {
    start(*stream_, options);
  }
  catch(...)
  {
    Debug::shutdown();
    throw;
  }
}

Recording::~Recording()
{
  if(stream_)
  {
    Debug::shutdown();
  }
}
} // namespace algorithms::viz
