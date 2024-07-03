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

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>

#include <boost/lexical_cast/bad_lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace nebula::ros::hw_monitor
{
class HttpLidarMonitorProvider : public DiagnosticProvider
{
public:
  std::map<std::string, hook_t> getHooks() override
  {
    return {{"hesai_voltage", [this](auto & diagnostics) { updateVoltage(diagnostics); }}};
  }

  void fetch(drivers::HesaiHwInterface & hw_interface, rclcpp::Clock & clock) override
  {
    hw_interface.GetLidarMonitorAsyncHttp([&](const std::string & str) {
      std::scoped_lock lock(mtx_lidar_monitor_);
      current_lidar_monitor_time_ = std::make_unique<rclcpp::Time>(clock.now());
      current_data_ = std::make_unique<boost::property_tree::ptree>(hw_interface.ParseJson(str));
    });
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
    if (!current_data_) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
      return;
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msgs;

    std::vector<std::string> keys = {"lidarInCur", "lidarInVol"};

    for (const auto & key : keys) {
      std::optional<std::string> value_opt{};
      std::string full_key = "Body." + key;

      try {
        auto result = current_data_->get_optional<std::string>(full_key);
        if (result.has_value()) {
          value_opt.emplace(result.value());
        } else {
          level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          msgs.emplace_back("field '" + full_key + "' not present");
        }
      } catch (boost::bad_lexical_cast & ex) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        msgs.emplace_back("field '" + full_key + "' not parsed: " + std::string(ex.what()));
      }

      if (value_opt.has_value()) {
        diagnostics.add(key, value_opt.value());
      }
    }

    diagnostics.summary(level, boost::algorithm::join(msgs, ", "));
  }

  std::unique_ptr<boost::property_tree::ptree> current_data_;
  std::unique_ptr<rclcpp::Time> current_lidar_monitor_time_;
  std::mutex mtx_lidar_monitor_;
};
}  // namespace nebula::ros::hw_monitor
