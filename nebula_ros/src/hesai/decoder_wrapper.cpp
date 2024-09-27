// Copyright 2024 TIER IV, Inc.

#include "nebula_ros/hesai/decoder_wrapper.hpp"

#include "cuda_blackboard/cuda_blackboard_publisher.hpp"
#include "cuda_blackboard/cuda_pointcloud2.hpp"

#include <nebula_common/hesai/hesai_common.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <mutex>
#include <utility>

#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
namespace nebula
{
namespace ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

HesaiDecoderWrapper::HesaiDecoderWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & config,
  const std::shared_ptr<const drivers::HesaiCalibrationConfigurationBase> & calibration,
  bool publish_packets)
: status_(nebula::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("HesaiDecoder")),
  parent_node_(*parent_node),
  sensor_cfg_(config),
  calibration_cfg_ptr_(calibration)
{
  if (!sensor_cfg_) {
    throw std::runtime_error("HesaiDecoderWrapper cannot be instantiated without a valid config!");
  }

  if (!calibration_cfg_ptr_) {
    throw std::runtime_error("HesaiDecoderWrapper cannot be instantiated without a valid config!");
  }

  RCLCPP_INFO_STREAM(
    logger_, "Using calibration data from " << calibration_cfg_ptr_->calibration_file);

  RCLCPP_INFO(logger_, "Starting Decoder");

  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(config, calibration_cfg_ptr_);
  status_ = driver_ptr_->GetStatus();

  if (Status::OK != status_) {
    throw std::runtime_error(
      (std::stringstream() << "Error instantiating decoder: " << status_).str());
  }

  // Publish packets only if enabled by the ROS wrapper
  if (publish_packets) {
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
    packets_pub_ = parent_node->create_publisher<pandar_msgs::msg::PandarScan>(
      "pandar_packets", rclcpp::SensorDataQoS());
  }

  cloud_pub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      parent_node_, "pandar_points");

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });
}

void HesaiDecoderWrapper::OnConfigChange(
  const std::shared_ptr<const nebula::drivers::HesaiSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::HesaiDriver>(new_config, calibration_cfg_ptr_);
  driver_ptr_ = new_driver;
  sensor_cfg_ = new_config;
}

void HesaiDecoderWrapper::OnCalibrationChange(
  const std::shared_ptr<const nebula::drivers::HesaiCalibrationConfigurationBase> & new_calibration)
{
  std::lock_guard lock(mtx_driver_ptr_);
  auto new_driver = std::make_shared<drivers::HesaiDriver>(sensor_cfg_, new_calibration);
  driver_ptr_ = new_driver;
  calibration_cfg_ptr_ = new_calibration;
}

void HesaiDecoderWrapper::ProcessCloudPacket(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (
    packets_pub_ && (packets_pub_->get_subscription_count() > 0 ||
                     packets_pub_->get_intra_process_subscription_count() > 0)) {
    if (current_scan_msg_->packets.size() == 0) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    pandar_msgs::msg::PandarPacket pandar_packet_msg{};
    pandar_packet_msg.stamp = packet_msg->stamp;
    pandar_packet_msg.size = packet_msg->data.size();
    std::copy(packet_msg->data.begin(), packet_msg->data.end(), pandar_packet_msg.data.begin());
    current_scan_msg_->packets.emplace_back(std::move(pandar_packet_msg));
  }

  std::unique_lock lock(mtx_driver_ptr_);
  auto [pointcloud, timestamp] = driver_ptr_->ParseCloudPacket(packet_msg->data);
  lock.unlock();

  // A pointcloud is only emitted when a scan completes (e.g. 3599 packets do not emit, the 3600th
  // emits one)
  if (pointcloud == nullptr) {
    // Since this ends the function early, the `cloud_watchdog_` will not be updated.
    // Thus, if pointclouds are not emitted for too long (e.g. when decoder settings are wrong or no
    // packets come in), the watchdog will log a warning automatically
    return;
  }

  cloud_watchdog_->update();

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty()) {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  }

  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  pointcloud->header.stamp = rclcpp::Time(SecondsToChronoNanoSeconds(timestamp).count());
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
  }

  if (
    !cloud_pub_->get_subscription_count() && !cloud_pub_->get_intra_process_subscription_count()) {
    return;
  }

  cloud_pub_->publish(std::move(pointcloud));
}

nebula::Status HesaiDecoderWrapper::Status()
{
  std::lock_guard lock(mtx_driver_ptr_);

  if (!driver_ptr_) {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->GetStatus();
}
}  // namespace ros
}  // namespace nebula
