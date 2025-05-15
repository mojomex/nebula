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

#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <chrono>
#include <functional>
#include <type_traits>
#include <utility>
#include <variant>

namespace nebula::ros
{

template <typename PayloadT = std::monostate>
class Watchdog
{
public:
  using timeout_callback_t = std::function<void()>;
  using tick_callback_t = std::function<void(PayloadT payload)>;

  explicit Watchdog(
    rclcpp::Node::SharedPtr node, rclcpp::Duration timeout, timeout_callback_t && on_timeout,
    tick_callback_t && on_tick)
  : on_tick_(std::move(on_tick)),
    on_timeout_(std::move(on_timeout)),
    timer_(node->create_wall_timer(
      timeout.to_chrono<std::chrono::nanoseconds>(), [this]() { on_timeout_internal(); }))
  {
  }

  std::enable_if_t<std::is_same_v<PayloadT, std::monostate>> tick() { tick({}); }

  void tick(PayloadT payload)
  {
    timer_->reset();
    if (on_tick_) on_tick_(std::move(payload));
  }

private:
  void on_timeout_internal()
  {
    if (on_timeout_) on_timeout_();
  }

  tick_callback_t on_tick_;
  timeout_callback_t on_timeout_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nebula::ros
