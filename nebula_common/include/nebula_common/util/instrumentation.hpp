#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>

namespace nebula
{
namespace util
{

class Instrumentation
{
public:
  Instrumentation(std::string tag)
  : tag(tag),
    last_print(std::chrono::duration_cast<std::chrono::nanoseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
                 .count())
  {
  }

  void tick()
  {
    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::high_resolution_clock::now().time_since_epoch())
                  .count();

    if (last_tick != 0) {
      auto rtt = now - last_tick;
      rtts_.update(rtt);
    }

    last_tick = now;
  }

  void tock()
  {
    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                 std::chrono::high_resolution_clock::now().time_since_epoch())
                 .count();
    auto delay = now - last_tick;
    delays_.update(delay);
    
    if (now - last_print >= PRINT_EVERY_NS) {
      uint64_t del_min, del_avg, del_max, rtt_min, rtt_avg, rtt_max, cnt;
      delays_.reset(del_min, del_avg, del_max, cnt);
      rtts_.reset(rtt_min, rtt_avg, rtt_max, cnt);

      last_print = now;

      auto frq_avg = 1e9 / rtt_avg;
      auto frq_min = 1e9 / rtt_max;
      auto frq_max = 1e9 / rtt_min;

      std::cout << "{\"tag\": \"" << tag
       << "\", \"del_min\": " << del_min * 1e-9
       << "\", \"del_avg\": " << del_avg * 1e-9
       << "\", \"del_max\": " << del_max * 1e-9
       << "\", \"rtt_min\": " << rtt_min * 1e-9
       << "\", \"rtt_avg\": " << rtt_avg * 1e-9
       << "\", \"rtt_max\": " << rtt_max * 1e-9
       << "\", \"frq_min\": " << frq_min
       << "\", \"frq_avg\": " << frq_avg
       << "\", \"frq_max\": " << frq_max
       << "\", \"window\": " << cnt << '}' << std::endl;
    }
  }

private:
  class Counter {
  public:
  void reset(uint64_t & ret_min, uint64_t & ret_avg, uint64_t & ret_max, uint64_t & ret_cnt) {
    auto avg_measurement = sum_measurements / num_measurements;

    ret_min = min_measurement;
    ret_avg = avg_measurement;
    ret_max = max_measurement;
    ret_cnt = num_measurements;

    sum_measurements = 0;
    num_measurements = 0;
    min_measurement = UINT64_MAX;
    max_measurement = 0;
  }

  void update(uint64_t measurement) {
    sum_measurements += measurement;
    num_measurements++;

    min_measurement = std::min(min_measurement, measurement);
    max_measurement = std::max(max_measurement, measurement);
  }

  private:
    uint64_t sum_measurements{0};
    uint64_t num_measurements{0};
    uint64_t min_measurement{UINT64_MAX};
    uint64_t max_measurement{0};
  };

  std::string tag;

  uint64_t last_tick{0};
  uint64_t last_print{0};

  Counter delays_;
  Counter rtts_;

  const uint64_t PRINT_EVERY_NS = 1'000'000'000;
};

}  // namespace util
}  // namespace nebula