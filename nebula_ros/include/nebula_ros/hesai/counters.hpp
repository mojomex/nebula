// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>

namespace nebula::ros
{
class PerformanceCounters
{
public:
  void declare(const std::string & counter) { counters_.try_emplace(counter, 0); }

  void incr(const std::string & counter, int64_t amount = 1)
  {
    counters_.try_emplace(counter, 0);
    counters_[counter] += amount;
  }

  PerformanceCounters & plus(const PerformanceCounters & other)
  {
    return plus_internal(other, false);
  }
  PerformanceCounters & minus(const PerformanceCounters & other)
  {
    return plus_internal(other, true);
  }

  void get_all(const std::function<void(const std::map<std::string, int64_t> &)> & callback) const
  {
    if (!callback) return;

    callback(counters_);
  }

  void reset() { counters_.clear(); }

private:
  PerformanceCounters & plus_internal(const PerformanceCounters & other, bool subtract)
  {
    for (const auto & [k, v] : other.counters_) {
      if (!counters_.count(k)) {
        counters_.try_emplace(k, 0);
      }

      if (subtract)
        counters_[k] -= v;
      else
        counters_[k] += v;
    }

    return *this;
  }

  std::map<std::string, int64_t> counters_;
};

}  // namespace nebula::ros
