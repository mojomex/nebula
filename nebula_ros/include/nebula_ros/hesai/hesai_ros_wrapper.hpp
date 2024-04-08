#pragma once

#include "boost_tcp_driver/tcp_driver.hpp"
#include "nebula_common/hesai/hesai_common.hpp"
#include "nebula_common/nebula_common.hpp"
#include "nebula_common/nebula_status.hpp"
// #include "nebula_common/util/instrumentation.hpp"
// #include "nebula_common/util/topic_delay.hpp"
#include "nebula_decoders/nebula_decoders_hesai/hesai_driver.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"
#include "nebula_ros/hesai/mt_queue.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "nebula_msgs/msg/nebula_packet.hpp"

#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#include <chrono>
#include <mutex>
#include <thread>
#include <array>

namespace nebula
{
namespace ros
{

/// @brief Get parameter from rclcpp::Parameter
/// @tparam T
/// @param p Parameter from rclcpp parameter callback
/// @param name Target parameter name
/// @param value Corresponding value
/// @return Whether the target name existed
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

/// @brief Ros wrapper of hesai driver
class HesaiRosWrapper final : public rclcpp::Node
{
public:
  explicit HesaiRosWrapper(const rclcpp::NodeOptions & options);
  ~HesaiRosWrapper();

  /// @brief Get current status of this driver
  /// @return Current status
  Status GetStatus();

  /// @brief Start point cloud streaming (Call CloudInterfaceStart of HwInterface)
  /// @return Resulting status
  Status StreamStart();

private:
  Status InitializeHwInterface();
  Status InitializeDecoder();
  Status InitializeHwMonitor();

  /// @brief Get configurations from ros parameters
  /// @param sensor_configuration Output of SensorConfiguration
  /// @return Resulting status
  Status GetParameters(drivers::HesaiSensorConfiguration & sensor_configuration);

  /// @brief Get calibration data from the sensor
  /// @param calibration_configuration Output of CalibrationConfiguration
  /// @param correction_configuration Output of CorrectionConfiguration (for AT)
  /// @return Resulting status
  Status GetCalibrationData(
    drivers::HesaiCalibrationConfiguration & calibration_configuration,
    drivers::HesaiCorrection & correction_configuration);

  /// @brief Convert seconds to chrono::nanoseconds
  /// @param seconds
  /// @return chrono::nanoseconds
  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  /***
   * Publishes a sensor_msgs::msg::PointCloud2 to the specified publisher
   * @param pointcloud unique pointer containing the point cloud to publish
   * @param publisher
   */
  void PublishCloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  /// @brief Callback for receiving a raw UDP packet
  /// @param scan_buffer Received PandarScan
  void ReceiveCloudPacketCallback(std::vector<uint8_t> & scan_buffer);

  /// @brief Decodes a nebula packet and, if it completes the scan, publishes the pointcloud.
  /// @param packet_msg The received packet message
  void ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  /// @brief rclcpp parameter callback
  /// @param parameters Received parameters
  /// @return SetParametersResult
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  /// @brief Updating rclcpp parameter
  /// @return SetParametersResult
  std::vector<rcl_interfaces::msg::SetParametersResult> updateParameters();

  /// @brief Initializing diagnostics
  void InitializeHesaiDiagnostics();
  /// @brief Callback of the timer for getting the current lidar status
  void OnHesaiStatusTimer();
  /// @brief Callback of the timer for getting the current lidar monitor via http
  void OnHesaiLidarMonitorTimerHttp();
  /// @brief Callback of the timer for getting the current lidar monitor via tcp
  void OnHesaiLidarMonitorTimer();
  //  void OnHesaiDiagnosticsTimer();
  //  void OnHesaiStatusTimer();

  /// @brief Check status information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckStatus(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check ptp information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckPtp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check temperature information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckTemperature(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check rpm information from HesaiLidarStatus for diagnostic_updater
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckRpm(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check voltage information from HesaiLidarStatus for diagnostic_updater via http
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckVoltageHttp(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Check voltage information from HesaiLidarStatus for diagnostic_updater via tcp
  /// @param diagnostics DiagnosticStatusWrapper
  void HesaiCheckVoltage(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  /// @brief Get value from property_tree
  /// @param pt property_tree
  /// @param key Pey string
  /// @return Value
  std::string GetPtreeValue(boost::property_tree::ptree * pt, const std::string & key);
  /// @brief Making fixed precision string
  /// @param val Target value
  /// @param pre Precision
  /// @return Created string
  std::string GetFixedPrecisionString(double val, int pre = 2);

  std::shared_ptr<drivers::HesaiDriver> driver_ptr_;
  drivers::HesaiHwInterface hw_interface_;

  Status wrapper_status_;
  Status interface_status_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_;

  std::shared_ptr<drivers::HesaiCalibrationConfiguration> calibration_cfg_ptr_;
  std::shared_ptr<drivers::HesaiSensorConfiguration> sensor_cfg_ptr_;
  std::shared_ptr<drivers::HesaiCorrection> correction_cfg_ptr_;

  /// @brief Stores received packets that have not been processed yet by the decoder thread
  mt_queue<std::unique_ptr<nebula_msgs::msg::NebulaPacket>> packet_queue_;
  /// @brief Thread to isolate decoding from receiving
  std::thread decoder_thread_;

  // todo: temporary class member during single node refactoring
  bool launch_hw_;

  // todo: temporary class member during single node refactoring
  std::string calibration_file_path;
  /// @brief File path of Correction data (Only required only for AT)
  std::string correction_file_path;

  /// @brief Received Hesai message publisher
  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr pandar_scan_pub_;

  bool retry_hw_;
  std::mutex mtx_config_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  diagnostic_updater::Updater diagnostics_updater_;

  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  rclcpp::TimerBase::SharedPtr fetch_diagnostics_timer_;

  std::unique_ptr<HesaiLidarStatus> current_status_;
  std::unique_ptr<HesaiLidarMonitor> current_monitor_;
  std::unique_ptr<HesaiConfig> current_config_;
  std::unique_ptr<HesaiInventory> current_inventory_;
  std::unique_ptr<boost::property_tree::ptree> current_lidar_monitor_tree_;

  std::unique_ptr<rclcpp::Time> current_status_time_;
  std::unique_ptr<rclcpp::Time> current_config_time_;
  std::unique_ptr<rclcpp::Time> current_inventory_time_;
  std::unique_ptr<rclcpp::Time> current_lidar_monitor_time_;

  uint8_t current_diag_status_;
  uint8_t current_monitor_status_;

  uint16_t diag_span_;
  std::mutex mtx_lidar_status_;
  std::mutex mtx_lidar_monitor_;

  std::string info_model_;
  std::string info_serial_;

  std::vector<std::string> temperature_names_;

  bool setup_sensor;
  const std::string MSG_NOT_SUPPORTED = "Not supported";
  const std::string MSG_ERROR = "Error";
  const std::string MSG_SEP = ": ";
};

}  // namespace ros
}  // namespace nebula
