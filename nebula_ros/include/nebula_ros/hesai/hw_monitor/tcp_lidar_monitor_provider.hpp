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

#include "nebula_ros/hesai/hw_monitor/diagnostic_provider.hpp"
#include "nebula_ros/hesai/hw_monitor/string_utils.hpp"

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <map>
#include <memory>
#include <string>

namespace nebula::ros::hw_monitor
{
class TcpLidarMonitorProvider : public DiagnosticProvider
{
public:
  std::map<std::string, hook_t> getHooks() override
  {
    return {{"hesai_voltage", [this](auto & diagnostics) { updateVoltage(diagnostics); }}};
  }

  void fetch(drivers::HesaiHwInterface & hw_interface, rclcpp::Clock & clock) override
  {
    auto result = hw_interface.GetLidarMonitor();
    std::scoped_lock lock(mtx_lidar_monitor_);
    current_lidar_monitor_time_ = std::make_unique<rclcpp::Time>(clock.now());
    current_monitor_ = std::make_unique<HesaiLidarMonitor>(result);
  }

  [[nodiscard]] std::optional<rclcpp::Time> getLastUpdate() const override
  {
    if (!current_lidar_monitor_time_) return {};
    return *current_lidar_monitor_time_;
  }

  [[nodiscard]] std::string getName() const override { return "LidarMonitor"; }

private:
  void updateVoltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
  {
    std::scoped_lock lock(mtx_lidar_monitor_);
    if (!current_monitor_) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
      return;
    }

    double value = current_monitor_->input_voltage.value() * 0.01;
    diagnostics.add("input_voltage", to_fixed(value, 3) + " V");

    value = current_monitor_->input_current.value() * 0.01;
    diagnostics.add("input_current", to_fixed(value, 3) + " mA");

    value = current_monitor_->input_power.value() * 0.01;
    diagnostics.add("input_power", to_fixed(value, 3) + " W");

    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  }

  std::unique_ptr<HesaiLidarMonitor> current_monitor_;
  std::unique_ptr<rclcpp::Time> current_lidar_monitor_time_;
  std::mutex mtx_lidar_monitor_;
};
}  // namespace nebula::ros::hw_monitor
