#pragma once

#include "nebula_decoders/nebula_decoders_velodyne/decoders/velodyne_scan_decoder.hpp"

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <array>
#include <memory>
#include <tuple>
#include <vector>

namespace nebula
{
namespace drivers
{
namespace vlp16
{
constexpr uint32_t MAX_POINTS = 300000;
/// @brief Velodyne LiDAR decoder (VLP16)
class Vlp16Decoder : public VelodyneScanDecoder
{
public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param calibration_configuration Calibration for this decoder
  explicit Vlp16Decoder(
    const std::shared_ptr<const drivers::VelodyneSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const drivers::VelodyneCalibrationConfiguration> & calibration_configuration);
  /// @brief Parsing and shaping VelodynePacket
  /// @param velodyne_packet
  void unpack(const std::vector<uint8_t> & packet, int32_t packet_seconds) override;
  /// @brief Calculation of points in each packet
  /// @return # of points
  int pointsPerPacket() override;
  /// @brief Get the constructed point cloud
  /// @return tuple of Point cloud and timestamp
  std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() override;
  /// @brief Resetting point cloud buffer
  void reset_pointcloud(double time_stamp) override;
  /// @brief Resetting overflowed point cloud buffer
  void reset_overflow(double time_stamp) override;

private:
  /// @brief Parsing VelodynePacket based on packet structure
  /// @param velodyne_packet
  /// @return Resulting flag
  bool parsePacket(const velodyne_msgs::msg::VelodynePacket & velodyne_packet) override;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];
  float rotation_radians_[ROTATION_MAX_UNITS];
  int phase_;
  int max_pts_;
  double last_block_timestamp_;
  std::vector<std::vector<float>> timing_offsets_;
};

}  // namespace vlp16
}  // namespace drivers
}  // namespace nebula
