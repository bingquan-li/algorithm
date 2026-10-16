#pragma once

#include "common.h"
#include <Eigen/Core>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace reference_line
{
// 首次日志前设置 REFERENCE_LINE_DEBUG_FILE 可同时追加 JSONL 文件；
// 未设置时仅输出 Rerun。文件打开或写入失败会抛出 std::runtime_error。
void
debug_log(std::string const& message);
void
error_log(std::string const& message);
template <typename... Args>
void
log_parts(bool error, Args&&... args)
{
  std::ostringstream stream;
  (stream << ... << std::forward<Args>(args));
  if(error)
  {
    error_log(stream.str());
  }
  else
  {
    debug_log(stream.str());
  }
}
void
log_xy_global_path(std::string const&, std::vector<Eigen::Vector3d> const&);
void
log_xy_path(std::string const&, std::vector<Eigen::Vector2d> const&);
void
log_xy_lane_boundary(std::string const&, std::vector<Eigen::Vector3d> const&);
void
log_xy_center_line(std::string const&, std::vector<Eigen::Vector3d> const&);
void
log_sl_left_boundary(std::vector<SLPoint> const&);
void
log_sl_right_boundary(std::vector<SLPoint> const&);
} // namespace reference_line

#define REFERENCE_DEBUG_LOG(...) ::reference_line::log_parts(false, __VA_ARGS__)
#define REFERENCE_ERROR_LOG(...) ::reference_line::log_parts(true, __VA_ARGS__)
