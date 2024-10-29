// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/hw_monitor_wrapper.hpp"

#include "nebula_ros/common/parameter_descriptors.hpp"

#include <nebula_common/nebula_common.hpp>
#include <nlohmann/json.hpp>

#include <diagnostic_msgs/msg/detail/diagnostic_status__struct.hpp>

#include <string>

namespace nebula::ros
{

using nlohmann::json;

void HesaiHwMonitorWrapper::add_json_item_to_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics, const std::string & key,
  const json & value)
{
  if (key.find("reserved") != std::string::npos) return;

  switch (value.type()) {
    case nlohmann::detail::value_t::string:
      diagnostics.add(key, value.template get<std::string>());
      break;
    default:
      diagnostics.add(key, value.dump());
  }
}

HesaiHwMonitorWrapper::HesaiHwMonitorWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::HesaiHwInterface> & hw_interface,
  std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config)
: logger_(parent_node->get_logger().get_child("HwMonitor")),
  diagnostics_updater_(parent_node),
  status_(Status::OK),
  hw_interface_(hw_interface),
  parent_node_(parent_node)
{
  diag_span_ = parent_node->declare_parameter<uint16_t>("diag_span", param_read_only());

  switch (config->sensor_model) {
    case nebula::drivers::SensorModel::HESAI_PANDARXT32:
    case nebula::drivers::SensorModel::HESAI_PANDARXT32M:
    case nebula::drivers::SensorModel::HESAI_PANDARAT128:
      temperature_names_.emplace_back("Bottom circuit board T1");
      temperature_names_.emplace_back("Bottom circuit board T2");
      temperature_names_.emplace_back("Laser emitting board RT_L1 (Internal)");
      temperature_names_.emplace_back("Laser emitting board RT_L2");
      temperature_names_.emplace_back("Receiving board RT_R");
      temperature_names_.emplace_back("Receiving board RT2");
      temperature_names_.emplace_back("Top circuit RT3");
      temperature_names_.emplace_back("Not used");
      break;
    case nebula::drivers::SensorModel::HESAI_PANDAR64:
    case nebula::drivers::SensorModel::HESAI_PANDAR40P:
    case nebula::drivers::SensorModel::HESAI_PANDAR40M:
    case nebula::drivers::SensorModel::HESAI_PANDARQT64:
    case nebula::drivers::SensorModel::HESAI_PANDARQT128:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E3X:
    case nebula::drivers::SensorModel::HESAI_PANDAR128_E4X:
    default:
      temperature_names_.emplace_back("Bottom circuit RT1");
      temperature_names_.emplace_back("Bottom circuit RT2");
      temperature_names_.emplace_back("Internal Temperature");
      temperature_names_.emplace_back("Laser emitting board RT1");
      temperature_names_.emplace_back("Laser emitting board RT2");
      temperature_names_.emplace_back("Receiving board RT1");
      temperature_names_.emplace_back("Top circuit RT1");
      temperature_names_.emplace_back("Top circuit RT2");
      break;
  }

  supports_monitor_ = config->sensor_model != drivers::SensorModel::HESAI_PANDARAT128;

  auto result = hw_interface->GetInventory();
  current_inventory_.reset(new HesaiInventory(result));
  current_inventory_time_.reset(new rclcpp::Time(parent_node->get_clock()->now()));
  std::cout << "HesaiInventory" << std::endl;
  std::cout << result << std::endl;
  info_model_ = result.get_str_model();
  info_serial_ = std::string(std::begin(result.sn), std::end(result.sn));
  RCLCPP_INFO_STREAM(logger_, "Model: " << info_model_);
  RCLCPP_INFO_STREAM(logger_, "Serial: " << info_serial_);
  initialize_hesai_diagnostics();
}

void HesaiHwMonitorWrapper::initialize_hesai_diagnostics()
{
  RCLCPP_INFO_STREAM(logger_, "initialize_hesai_diagnostics");
  using std::chrono_literals::operator""s;
  std::ostringstream os;
  auto hardware_id = info_model_ + ": " + info_serial_;
  diagnostics_updater_.setHardwareID(hardware_id);
  RCLCPP_INFO_STREAM(logger_, "Hardware ID: " + hardware_id);

  diagnostics_updater_.add("hesai_status", this, &HesaiHwMonitorWrapper::hesai_check_status);
  diagnostics_updater_.add("hesai_ptp", this, &HesaiHwMonitorWrapper::hesai_check_ptp);
  diagnostics_updater_.add(
    "hesai_temperature", this, &HesaiHwMonitorWrapper::hesai_check_temperature);
  diagnostics_updater_.add("hesai_rpm", this, &HesaiHwMonitorWrapper::hesai_check_rpm);

  current_status_.reset();
  current_monitor_.reset();
  current_status_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
  current_lidar_monitor_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
  current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  current_monitor_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;

  auto fetch_diag_from_sensor = [this]() {
    on_hesai_status_timer();

    if (!supports_monitor_) return;

    if (hw_interface_->UseHttpGetLidarMonitor()) {
      on_hesai_lidar_monitor_timer_http();
    } else {
      on_hesai_lidar_monitor_timer();
    }
  };

  fetch_diagnostics_timer_ = parent_node_->create_wall_timer(
    std::chrono::milliseconds(diag_span_), std::move(fetch_diag_from_sensor));

  if (hw_interface_->UseHttpGetLidarMonitor()) {
    diagnostics_updater_.add(
      "hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage_http);
  } else {
    diagnostics_updater_.add("hesai_voltage", this, &HesaiHwMonitorWrapper::hesai_check_voltage);
  }

  auto on_timer_update = [this] {
    RCLCPP_DEBUG_STREAM(logger_, "OnUpdateTimer");
    auto now = parent_node_->get_clock()->now();
    auto dif = (now - *current_status_time_).seconds();

    RCLCPP_DEBUG_STREAM(logger_, "dif(status): " << dif);

    if (diag_span_ * 2.0 < dif * 1000) {
      current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(logger_, "STALE");
    } else {
      current_diag_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(logger_, "OK");
    }
    dif = (now - *current_lidar_monitor_time_).seconds();
    RCLCPP_DEBUG_STREAM(logger_, "dif(monitor): " << dif);
    if (diag_span_ * 2.0 < dif * 1000) {
      current_monitor_status_ = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      RCLCPP_DEBUG_STREAM(logger_, "STALE");
    } else {
      current_monitor_status_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
      RCLCPP_DEBUG_STREAM(logger_, "OK");
    }
    diagnostics_updater_.force_update();
  };
  diagnostics_update_timer_ =
    parent_node_->create_wall_timer(std::chrono::milliseconds(1000), std::move(on_timer_update));

  RCLCPP_DEBUG_STREAM(logger_, "add_timer");
}

std::string HesaiHwMonitorWrapper::get_ptree_value(
  boost::property_tree::ptree * pt, const std::string & key)
{
  boost::optional<std::string> value = pt->get_optional<std::string>(key);
  if (value) {
    return value.get();
  } else {
    return MSG_NOT_SUPPORTED_;
  }
}
std::string HesaiHwMonitorWrapper::get_fixed_precision_string(double val, int pre)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(pre) << val;
  return ss.str();
}

void HesaiHwMonitorWrapper::on_hesai_status_timer()
{
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_status_timer" << std::endl);
  try {
    auto result = hw_interface_->GetLidarStatus();
    std::scoped_lock lock(mtx_lidar_status_);
    current_status_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
    current_status_ = result;
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiHwMonitorWrapper::on_hesai_status_timer(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_status_timer(boost::system::system_error)"),
      error.what());
  }
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_status_timer END" << std::endl);
}

void HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer_http()
{
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer_http");
  try {
    hw_interface_->GetLidarMonitorAsyncHttp([this](const std::string & str) {
      std::scoped_lock lock(mtx_lidar_monitor_);
      current_lidar_monitor_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
      current_lidar_monitor_tree_ =
        std::make_unique<boost::property_tree::ptree>(hw_interface_->ParseJson(str));
    });
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer_http(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer_http(boost::system::system_"
        "error)"),
      error.what());
  }
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer_http END");
}

void HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer()
{
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer");
  try {
    auto result = hw_interface_->GetLidarMonitor();
    std::scoped_lock lock(mtx_lidar_monitor_);
    current_lidar_monitor_time_.reset(new rclcpp::Time(parent_node_->get_clock()->now()));
    current_monitor_.reset(new HesaiLidarMonitor_OT128(result));
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer(std::system_error)"),
      error.what());
  } catch (const boost::system::system_error & error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "HesaiHwMonitorWrapper::on_hesai_lidar_monitor_timer(boost::system::system_"
        "error)"),
      error.what());
  }
  RCLCPP_DEBUG_STREAM(logger_, "on_hesai_lidar_monitor_timer END");
}

void HesaiHwMonitorWrapper::hesai_check_status(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    json data = current_status_->to_json();
    for (const auto & [key, value] : data.items()) {
      if (
        key == "motor_speed" || key == "temperature" ||
        (key.find("ptp") != std::string::npos || key.find("gps") != std::string::npos)) {
        continue;
      }

      add_json_item_to_diagnostics(diagnostics, key, value);
    }
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_ptp(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    json data = current_status_->to_json();
    for (const auto & [key, value] : data.items()) {
      if (key.find("ptp") == std::string::npos && key.find("gps") == std::string::npos) {
        continue;
      }

      add_json_item_to_diagnostics(diagnostics, key, value);
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_temperature(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::vector<std::string> msg;
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    json data = current_status_->to_json();
    if (data.contains("temperature")) {
      for (const auto & [key, value] : data["temperature"].items()) {
        add_json_item_to_diagnostics(diagnostics, key, value);
      }
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_rpm(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_status_);
  if (current_status_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    json data = current_status_->to_json();
    if (data.contains("motor_speed")) {
      add_json_item_to_diagnostics(diagnostics, "motor_speed", data["motor_speed"]);
    }
    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_voltage_http(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor_);
  if (current_lidar_monitor_tree_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    std::string key = "";

    std::string mes;
    key = "lidarInCur";
    try {
      mes = get_ptree_value(current_lidar_monitor_tree_.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = MSG_ERROR_ + std::string(ex.what());
    }
    add_json_item_to_diagnostics(diagnostics, key, mes);
    key = "lidarInVol";
    try {
      mes = get_ptree_value(current_lidar_monitor_tree_.get(), "Body." + key);
    } catch (boost::bad_lexical_cast & ex) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      mes = MSG_ERROR_ + std::string(ex.what());
    }
    add_json_item_to_diagnostics(diagnostics, key, mes);

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

void HesaiHwMonitorWrapper::hesai_check_voltage(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::scoped_lock lock(mtx_lidar_monitor_);
  if (current_monitor_) {
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> msg;
    json data = current_monitor_->to_json();
    for (const auto & [key, value] : data.items()) {
      add_json_item_to_diagnostics(diagnostics, key, value);
    }

    diagnostics.summary(level, boost::algorithm::join(msg, ", "));
  } else {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data available");
  }
}

Status HesaiHwMonitorWrapper::status()
{
  return Status::OK;
}
}  // namespace nebula::ros
