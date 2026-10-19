#pragma once
#include "../statistics.hh"
#include <chrono>
#include <stdexcept>

namespace algorithms::local_escape_detail
{
using SearchClock = std::chrono::steady_clock;

// 仅收集统计，不打印。异常展开时也将已收集的数据写回调用方。
class SearchTiming : public SearchStatistics
{
public:
  explicit SearchTiming(SearchStatistics& output)
    : output_(output)
  {
    output = {};
  }
  SearchTiming(SearchTiming const&) = delete;
  SearchTiming&
  operator=(SearchTiming const&) = delete;
  ~SearchTiming()
  {
    auto const end = SearchClock::now();
    finish_phase(end);
    total_milliseconds =
        std::chrono::duration<double, std::milli>(end - begin_).count();
    output_ = static_cast<SearchStatistics const&>(*this);
  }
  void
  phase(std::size_t index)
  {
    if(index >= phases.size())
    {
      throw std::out_of_range("Invalid search timing phase");
    }
    finish_phase(SearchClock::now());
    phase_index_ = index;
  }

private:
  void
  finish_phase(SearchClock::time_point now)
  {
    // phase_index_ 初始为零，后续更新经过 phase() 校验。
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    auto& entry = phases[phase_index_];
    entry.milliseconds +=
        std::chrono::duration<double, std::milli>(now - phase_begin_).count();
    ++entry.calls;
    phase_begin_ = now;
  }
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  SearchStatistics& output_;
  SearchClock::time_point begin_ = SearchClock::now();
  SearchClock::time_point phase_begin_ = begin_;
  std::size_t phase_index_ = 0;
};

class DetailTimer
{
public:
  explicit DetailTimer(TimingEntry& entry)
    : entry_(entry)
    , begin_(SearchClock::now())
  {}
  ~DetailTimer()
  {
    entry_.milliseconds +=
        std::chrono::duration<double, std::milli>(SearchClock::now() - begin_)
            .count();
    ++entry_.calls;
  }
  DetailTimer(DetailTimer const&) = delete;
  DetailTimer&
  operator=(DetailTimer const&) = delete;
  DetailTimer(DetailTimer&&) = delete;
  DetailTimer&
  operator=(DetailTimer&&) = delete;

private:
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  TimingEntry& entry_;
  SearchClock::time_point begin_;
};
} // namespace algorithms::local_escape_detail
