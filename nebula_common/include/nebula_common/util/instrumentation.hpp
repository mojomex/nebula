#pragma once

#include "nebula_common/util/performance_counter.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>

namespace nebula
{
namespace util
{

using namespace std::chrono_literals;

class Instrumentation
{
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  using duration = std::chrono::nanoseconds;

public:
  Instrumentation(std::string tag)
  : tag_(std::move(tag)), last_print_(clock::now()), delays_(), rtts_()
  {
  }

  void tick()
  {
    auto now = clock::now();

    if (last_tick_ != time_point::min()) {
      auto rtt = now - last_tick_;
      rtts_.update(rtt);
    }

    last_tick_ = now;
  }

  void tock()
  {
    auto now = clock::now();
    auto delay = now - last_tick_;
    delays_.update(delay);

    if (now - last_print_ >= PRINT_INTERVAL) {
      auto del = delays_.reset();
      auto rtt = rtts_.reset();

      last_print_ = now;

      std::stringstream ss;

      ss << "{\"type\": \"instrumentation\", \"tag\": \"" << tag_
         << "\", \"del\": [";
      
      for (duration & dt : *del) {
        ss << dt.count() << ',';
      }

      ss <<"null], \"rtt\": [";

      for (duration & dt : *rtt) {
        ss << dt.count() << ',';
      }
         
      ss <<  "null]}" << std::endl;

      std::cout << ss.str();
    }
  }

private:
  std::string tag_;

  time_point last_tick_{time_point::min()};
  time_point last_print_{time_point::min()};

  Counter delays_;
  Counter rtts_;

  const duration PRINT_INTERVAL = 1s;
};

}  // namespace util
}  // namespace nebula