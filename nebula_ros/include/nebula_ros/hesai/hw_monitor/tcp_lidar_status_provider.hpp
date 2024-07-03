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

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros::hw_monitor
{
class TcpLidarStatusProvider : public DiagnosticProvider
{
public:
  explicit TcpLidarStatusProvider(std::vector<std::string> temperature_names)
  : temperature_names_(std::move(temperature_names))
  {
  }

  std::map<std::string, hook_t> getHooks() override
  {
    std::map<std::string, hook_t> hooks;
    hooks.insert({"hesai_status", [this](auto & diagnostics) { updateStatus(diagnostics); }});
    hooks.insert({"hesai_ptp", [this](auto & diagnostics) { updatePtp(diagnostics); }});
    hooks.insert(
      {"hesai_temperature", [this](auto & diagnostics) { updateTemperature(diagnostics); }});
    hooks.insert({"hesai_rpm", [this](auto & diagnostics) { updateRpm(diagnostics); }});

    return hooks;
  }

  void fetch(drivers::HesaiHwInterface & hw_interface, rclcpp::Clock & clock) override
  {
    auto result = hw_interface.GetLidarStatus();
    std::scoped_lock lock(mtx_lidar_status_);
    current_status_time_ = std::make_unique<rclcpp::Time>(clock.now());
    current_status_ = std::make_unique<HesaiLidarStatus>(result);
  }

  [[nodiscard]] std::optional<rclcpp::Time> getLastUpdate() const override
  {
    if (!current_status_time_) return {};
    return *current_status_time_;
  }

  [[nodiscard]] std::string getName() const override { return "LidarStatus"; }

private:
  void updateStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
  {
    std::scoped_lock lock(mtx_lidar_status_);
    if (!current_status_) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
      return;
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    diagnostics.add("system_uptime", std::to_string(current_status_->system_uptime.value()));
    diagnostics.add("startup_times", std::to_string(current_status_->startup_times.value()));
    diagnostics.add(
      "total_operation_time", std::to_string(current_status_->total_operation_time.value()));

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  }

  void updatePtp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
  {
    std::scoped_lock lock(mtx_lidar_status_);
    if (!current_status_) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
      return;
    }

    auto gps_status = current_status_->get_str_gps_pps_lock();
    auto gprmc_status = current_status_->get_str_gps_gprmc_status();
    auto ptp_status = current_status_->get_str_ptp_clock_status();

    std::transform(gps_status.cbegin(), gps_status.cend(), gps_status.begin(), toupper);
    std::transform(gprmc_status.cbegin(), gprmc_status.cend(), gprmc_status.begin(), toupper);
    std::transform(ptp_status.cbegin(), ptp_status.cend(), ptp_status.begin(), toupper);

    diagnostics.add("gps_pps_lock", gps_status);
    diagnostics.add("gps_gprmc_status", gprmc_status);
    diagnostics.add("ptp_clock_status", ptp_status);

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;

    if (gps_status != "UNKNOWN") {
      msg.emplace_back("gps_pps_lock: " + gps_status);
    }
    if (gprmc_status != "UNKNOWN") {
      msg.emplace_back("gprmc_status: " + gprmc_status);
    }
    if (ptp_status != "UNKNOWN") {
      msg.emplace_back("ptp_status: " + ptp_status);
    }
    if (ptp_status == "FREE RUN" && gps_status == "UNKNOWN") {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  }

  void updateTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
  {
    std::scoped_lock lock(mtx_lidar_status_);
    if (!current_status_) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
      return;
    }

    for (size_t i = 0; i < std::size(current_status_->temperature); i++) {
      const auto & key = temperature_names_[i];
      auto value = current_status_->temperature[i].value() * 0.01;
      diagnostics.add(key, to_fixed(value, 3));
    }

    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  }

  void updateRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
  {
    std::scoped_lock lock(mtx_lidar_status_);
    if (!current_status_) {
      diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
      return;
    }

    diagnostics.add("motor_speed", std::to_string(current_status_->motor_speed.value()));
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  }

  std::unique_ptr<HesaiLidarStatus> current_status_;
  std::unique_ptr<rclcpp::Time> current_status_time_;
  std::mutex mtx_lidar_status_;

  const std::vector<std::string> temperature_names_;
};
}  // namespace nebula::ros::hw_monitor
