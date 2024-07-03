// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hw_monitor_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"
#include "nebula_ros/hesai/hw_monitor/diagnostic_provider.hpp"
#include "nebula_ros/hesai/hw_monitor/http_lidar_monitpr_provider.hpp"
#include "nebula_ros/hesai/hw_monitor/tcp_lidar_monitor_provider.hpp"
#include "nebula_ros/hesai/hw_monitor/tcp_lidar_status_provider.hpp"

#include <nebula_common/nebula_common.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nebula
{
namespace ros
{
HesaiHwMonitorWrapper::HesaiHwMonitorWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::HesaiHwInterface> & hw_interface,
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  diagnostics_updater_(parent_node),
  status_(Status::OK),
  hw_interface_(hw_interface),
  parent_node_(parent_node),
  diagnostic_providers_(getDiagnosticProviders(config->sensor_model))
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());
  initializeHesaiDiagnostics();
  RCLCPP_INFO_STREAM(logger_, "Initialized hardware monitor");
}

std::vector<std::shared_ptr<hw_monitor::DiagnosticProvider>>
HesaiHwMonitorWrapper::getDiagnosticProviders(drivers::SensorModel sensor_model)
{
  using drivers::SensorModel;

  std::vector<std::shared_ptr<hw_monitor::DiagnosticProvider>> result;

  std::vector<std::string> temperature_names;

  switch (sensor_model) {
    case SensorModel::HESAI_PANDARXT32:
    case SensorModel::HESAI_PANDARXT32M:
    case SensorModel::HESAI_PANDARAT128:
      temperature_names.emplace_back("Bottom circuit board T1");
      temperature_names.emplace_back("Bottom circuit board T2");
      temperature_names.emplace_back("Laser emitting board RT_L1 (Internal)");
      temperature_names.emplace_back("Laser emitting board RT_L2");
      temperature_names.emplace_back("Receiving board RT_R");
      temperature_names.emplace_back("Receiving board RT2");
      temperature_names.emplace_back("Top circuit RT3");
      temperature_names.emplace_back("Not used");
      break;
    case SensorModel::HESAI_PANDAR64:
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
    case SensorModel::HESAI_PANDARQT64:
    case SensorModel::HESAI_PANDARQT128:
    case SensorModel::HESAI_PANDAR128_E3X:
    case SensorModel::HESAI_PANDAR128_E4X:
      temperature_names.emplace_back("Bottom circuit RT1");
      temperature_names.emplace_back("Bottom circuit RT2");
      temperature_names.emplace_back("Internal Temperature");
      temperature_names.emplace_back("Laser emitting board RT1");
      temperature_names.emplace_back("Laser emitting board RT2");
      temperature_names.emplace_back("Receiving board RT1");
      temperature_names.emplace_back("Top circuit RT1");
      temperature_names.emplace_back("Top circuit RT2");
      break;
    default:
      throw std::runtime_error("Sensor not supported");
  }

  auto status_provider =
    std::make_shared<hw_monitor::TcpLidarStatusProvider>(std::move(temperature_names));
  result.push_back(status_provider);

  if (sensor_model == drivers::SensorModel::HESAI_PANDARAT128) {
    return result;
  }

  auto use_http_monitor =
    hw_interface_->UseHttpGetLidarMonitor(hw_interface_->NebulaModelToHesaiModelNo(sensor_model));

  if (use_http_monitor) {
    auto monitor = std::make_shared<hw_monitor::HttpLidarMonitorProvider>();
    result.push_back(monitor);
  } else {
    auto monitor = std::make_shared<hw_monitor::TcpLidarMonitorProvider>();
    result.push_back(monitor);
  }

  return result;
}

void HesaiHwMonitorWrapper::initializeHesaiDiagnostics()
{
  using std::chrono_literals::operator""s;
  std::ostringstream os;

  auto result = hw_interface_->GetInventory();
  auto model = result.get_str_model();
  auto serial = std::string(std::begin(result.sn), std::end(result.sn));
  auto hardware_id = model + ": " + serial;

  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(logger_, "hardware_id: " + hardware_id);

  for (auto & provider : diagnostic_providers_) {
    RCLCPP_INFO_STREAM(logger_, "Initializing diagnostic provider: " + provider->getName());
    auto hooks = provider->getHooks();
    for (const auto & entry : hooks) {
      diagnostics_updater_.add(entry.first, entry.second);
      RCLCPP_INFO_STREAM(logger_, "  Added diagnostic " + entry.first);
    }
  }

  auto fetch_diag_from_sensor = [this]() {
    for (auto & provider : diagnostic_providers_) {
      provider->fetch(*hw_interface_, *parent_node_->get_clock());
    }
  };

  fetch_diagnostics_timer_ = parent_node_->create_wall_timer(
    std::chrono::milliseconds(diag_span_), std::move(fetch_diag_from_sensor));

  auto on_timer_update = [this] {
    RCLCPP_DEBUG_STREAM(logger_, "OnUpdateTimer");
    auto now = parent_node_->get_clock()->now();

    for (auto & provider : diagnostic_providers_) {
      auto last_update = provider->getLastUpdate();
      bool stale{};
      if (!last_update.has_value()) {
        stale = true;
      } else {
        auto diff = (now - *last_update).seconds();
        stale = diag_span_ * 2.0 < diff * 1000;
      }

      if (stale) {
        RCLCPP_WARN_STREAM(logger_, "Diagnostic data for " + provider->getName() + " is stale.");
      } else {
        RCLCPP_DEBUG_STREAM(logger_, "Diagnostic data for " + provider->getName() + " is ok.");
      }
    }

    diagnostics_updater_.force_update();
  };

  diagnostics_update_timer_ =
    parent_node_->create_wall_timer(std::chrono::milliseconds(1000), std::move(on_timer_update));
}

Status HesaiHwMonitorWrapper::Status()
{
  return Status::OK;
}
}  // namespace ros
}  // namespace nebula
