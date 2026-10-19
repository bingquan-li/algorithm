#pragma once
#include "statistics.hh"
#include <chrono>
#include <cstdio>
#include <utility>

namespace narrow_passage
{
// steady_clock 不受系统时间调整影响；异常退出作用域时也记录耗时。
class ScopedTimer
{
public:
  explicit ScopedTimer(char const* label)
    : label_(label)
    , start_(std::chrono::steady_clock::now())
  {}

  ~ScopedTimer()
  {
    double const milliseconds = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - start_)
                                    .count();
    std::fprintf(stderr, "[timing] %s: %.3f ms\n", label_, milliseconds);
  }

  ScopedTimer(ScopedTimer const&) = delete;
  ScopedTimer&
  operator=(ScopedTimer const&) = delete;

private:
  char const* label_;
  std::chrono::steady_clock::time_point start_;
};

template <typename Function>
decltype(auto)
timed_call(char const* label, Function&& function)
{
  ScopedTimer const timer(label);
  return std::forward<Function>(function)();
}

inline void
print_search_statistics(algorithms::SearchStatistics const& stats)
{
  std::fprintf(stderr,
               "[search timing] plan total: %.3f ms\n",
               stats.total_milliseconds);
  auto const* name = algorithms::SearchStatistics::phase_names.begin();
  for(auto const& phase : stats.phases)
  {
    std::fprintf(stderr,
                 "[search timing] phase %-38s %.3f ms%s\n",
                 *name++,
                 phase.milliseconds,
                 (phase.calls != 0U) ? "" : " (skipped)");
  }
  auto const* detail_name = algorithms::SearchStatistics::detail_names.begin();
  for(auto const& detail : stats.details)
  {
    std::fprintf(stderr,
                 "[search timing] detail %-24s %.3f ms, calls=%zu, avg=%.6f ms "
                 "(inclusive)\n",
                 *detail_name++,
                 detail.milliseconds,
                 detail.calls,
                 (detail.calls != 0U) ? detail.milliseconds / detail.calls
                                      : 0.0);
  }
  std::fprintf(stderr,
               "[search timing] cell_distance calls=%zu\n",
               stats.cell_distance_calls);
}
} // namespace narrow_passage
