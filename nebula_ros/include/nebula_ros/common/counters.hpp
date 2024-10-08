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

#include "nebula_ros/common/debug_publisher.hpp"

#include <tier4_debug_msgs/msg/int64_stamped.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
namespace nebula::ros
{

class Counter
{
public:
  using clock = std::chrono::steady_clock;

  explicit Counter(rclcpp::Node * node) : debug_pub_(node, node->get_fully_qualified_name()) {}

  void contribute(const std::string & key, clock::duration duration)
  {
    measurements_.try_emplace(key, 0);
    measurements_[key] += duration;
  }

  bool try_publish(const std::string & key)
  {
    if (measurements_.find(key) == measurements_.end()) {
      return false;
    }

    debug_pub_.publish<tier4_debug_msgs::msg::Int64Stamped>(key, measurements_[key].count());
    measurements_.erase(key);
    return true;
  }

  void publish_all()
  {
    for (const auto & [key, value] : measurements_) {
      debug_pub_.publish<tier4_debug_msgs::msg::Int64Stamped>(key, value.count());
    }
    measurements_.clear();
  }

private:
  autoware::universe_utils::DebugPublisher debug_pub_;
  std::map<std::string, clock::duration> measurements_;
};

}  // namespace nebula::ros
