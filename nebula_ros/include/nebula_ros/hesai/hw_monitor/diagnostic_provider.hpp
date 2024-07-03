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

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp>
#include <rclcpp/time.hpp>

#include <functional>
#include <map>
#include <string>

namespace nebula::ros::hw_monitor
{

class DiagnosticProvider
{
public:
  using hook_t = std::function<void(diagnostic_updater::DiagnosticStatusWrapper &)>;

  virtual std::map<std::string, hook_t> getHooks() = 0;
  virtual void fetch(drivers::HesaiHwInterface & hw_interface, rclcpp::Clock & clock) = 0;

  [[nodiscard]] virtual std::optional<rclcpp::Time> getLastUpdate() const = 0;

  [[nodiscard]] virtual std::string getName() const = 0;
};

}  // namespace nebula::ros::hw_monitor
