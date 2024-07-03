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

#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_cmd_response.hpp"
#include "nebula_ros/hesai/hw_monitor/diagnostic_provider.hpp"
#include "nebula_ros/hesai/hw_monitor/string_utils.hpp"

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <map>
#include <memory>
#include <string>

namespace nebula::ros::hw_monitor
{
class TcpFaultModeInfoProvider : public DiagnosticProvider
{
public:
  std::map<std::string, hook_t> getHooks() override
  {
    return {{"fault_info", [this](auto & diagnostics) { updateFaultInfo(diagnostics); }}};
  }

  void fetch(drivers::HesaiHwInterface & hw_interface, rclcpp::Clock & clock) override
  {
    auto result = hw_interface.GetFaultModeInfo();
    std::scoped_lock lock(mtx_data_);
    last_update_ = std::make_unique<rclcpp::Time>(clock.now());
    current_data_ = std::make_unique<HesaiFaultModeInfo>(result);
  }

  [[nodiscard]] std::optional<rclcpp::Time> getLastUpdate() const override
  {
    if (!last_update_) return {};
    return *last_update_;
  }

  [[nodiscard]] std::string getName() const override { return "LidarMonitor"; }

private:
  void updateFaultInfo(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
  {
    std::scoped_lock lock(mtx_data_);
    if (!current_data_) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
      return;
    }

    diagnostics.add("operation_mode", current_data_->describeWorkMode());
    diagnostics.add("faults", current_data_->describeFaultCode());

    if (current_data_->ok()) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
      return;
    }

    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "");
  }

  std::unique_ptr<HesaiFaultModeInfo> current_data_;
  std::unique_ptr<rclcpp::Time> last_update_;
  std::mutex mtx_data_;
};
}  // namespace nebula::ros::hw_monitor
