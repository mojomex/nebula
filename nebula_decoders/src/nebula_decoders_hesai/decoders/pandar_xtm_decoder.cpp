#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xtm_decoder.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xtm.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_xtm
{
PandarXTMDecoder::PandarXTMDecoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elevation_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }
  /////////////////

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elevation_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
    elevation_angle_rad_[laser] = deg2rad(elevation_angle_[laser]);
    azimuth_offset_rad_[laser] = deg2rad(azimuth_offset_[laser]);
  }

  sin_elevation_angle_.resize(LASER_COUNT);
  cos_elevation_angle_.resize(LASER_COUNT);
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    sin_elevation_angle_[laser] = sinf(elevation_angle_rad_[laser]);
    cos_elevation_angle_[laser] = cosf(elevation_angle_rad_[laser]);
  }
  for (int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    block_azimuth_rad_[i] = deg2rad(i / 100.);
  }
  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_timestamp_ = -1;

  scan_pc_.reset(new NebulaPointCloud);
  scan_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new NebulaPointCloud);
  overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
}

bool PandarXTMDecoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> PandarXTMDecoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, scan_timestamp_);
}

int_least32_t PandarXTMDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return -1;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new NebulaPointCloud);
    overflow_pc_->reserve(LASER_COUNT * MAX_AZIMUTH_STEPS);
    has_scanned_ = false;
  }

  bool is_dual = packet_.return_mode == DUAL_RETURN || packet_.return_mode == DUAL_RETURN_B ||
                 packet_.return_mode == DUAL_RETURN_C;

  for (int block_id = 0; block_id < packet_.header.chBlockNumber; block_id += (is_dual ? 2 : 1)) {
    int current_phase =
      (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    if (current_phase < last_azimuth_ || has_scanned_) {
      auto block_pc = is_dual ? convert_dual(block_id) : convert(block_id);
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    } else {
      auto block_pc = is_dual ? convert_dual(block_id) : convert(block_id);
      *scan_pc_ += *block_pc;
    }
    last_azimuth_ = current_phase;
    last_timestamp_ = packet_.usec;
  }
  return last_azimuth_;
}

#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
void PandarXTMDecoder::CalcXTPointXYZIT(
  int block_id, char i, boost::shared_ptr<pcl::PointCloud<NebulaPoint>> cld)
{
#else
void PandarXTMDecoder::CalcXTPointXYZIT(
  int block_id, char i, std::shared_ptr<pcl::PointCloud<NebulaPoint>> cld)
{
#endif
  Block * block = &packet_.blocks[block_id];
  auto unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)
  /* for all the units in a block */
  const Unit & unit = block->units[i];
  NebulaPoint point{};

  /* skip invalid points */
  if (unit.distance < MIN_RANGE || unit.distance > MAX_RANGE) {
    return;
  }

  int azimuth = static_cast<int>(azimuth_offset_[i] * 100 + block->azimuth);
  if (azimuth < 0) azimuth += 36000;
  if (azimuth >= 36000) azimuth -= 36000;

  point.distance = unit.distance;
  point.azimuth = block_azimuth_rad_[block->azimuth] + azimuth_offset_rad_[i];
  point.elevation = elevation_angle_rad_[i];

  float xyDistance = unit.distance * cos_elevation_angle_[i];
  point.x = xyDistance * sinf(point.azimuth);
  point.y = xyDistance * cosf(point.azimuth);
  point.z = unit.distance * sin_elevation_angle_[i];

  point.intensity = unit.intensity;

  if (scan_timestamp_ < 0) {  // invalid timestamp
    scan_timestamp_ = unix_second + static_cast<double>(packet_.usec) / 1000000.f;
  }
  double offset;
  bool is_dual_return = false;

  if (packet_.return_mode == TRIPLE_RETURN) {
      offset =
        (static_cast<double>(blockXTMOffsetTriple[block_id] + laserXTMOffset[i]) / 1000000.0f);
  } else if (
    packet_.return_mode == DUAL_RETURN || packet_.return_mode == DUAL_RETURN_B ||
    packet_.return_mode == DUAL_RETURN_C) {
    is_dual_return = true;
    offset = (static_cast<double>(blockXTMOffsetDual[block_id] + laserXTMOffset[i]) / 1000000.0f);
  } else {
    offset =
      (static_cast<double>(blockXTMOffsetSingle[block_id] + laserXTMOffset[i]) / 1000000.0f);
  }

  // Filter out pairs of close points in dual return mode (keep the odd block_id)
  if (is_dual_return && block_id % 2 == 0) {
    Block * other_block = &packet_.blocks[block_id + 1];
    const Unit & other_unit = other_block->units[i];

    if (other_unit.distance == unit.distance && other_unit.intensity == unit.intensity) {
      return;
    }

    if (std::abs(other_unit.distance - unit.distance) < dual_return_distance_threshold_) {
      return;
    }
  }

  //    point.return_type = packet_.return_mode;
  switch (packet_.return_mode) {
    case DUAL_RETURN:  // Dual Return (Last, Strongest)
      if (i % 2 == 0) {
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::LAST);
      } else {
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
      }
      break;

    case DUAL_RETURN_B:  // Dual Return (Last, First)
      if (i % 2 == 0) {
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::LAST);
      } else {
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::FIRST);
      }
      break;

    case DUAL_RETURN_C:  // Dual Return (First, Strongest)
      if (i % 2 == 0) {
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::FIRST);
      } else {
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
      }
      break;

    default:
      point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::UNKNOWN);
      break;
  }
  auto point_stamp =
    (unix_second - offset + static_cast<double>(packet_.usec) / 1000000.f - scan_timestamp_);
  if (point_stamp < 0) {
    point.time_stamp = 0;
  } else {
    point.time_stamp = static_cast<uint32_t>(point_stamp * 10e9);
  }
  point.channel = i;
  cld->points.emplace_back(point);
}

drivers::NebulaPointCloudPtr PandarXTMDecoder::convert(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);
  for (int i = 0; i < packet_.header.chLaserNumber; ++i) {
    CalcXTPointXYZIT(block_id, i, block_pc);
  }

  return block_pc;
}

drivers::NebulaPointCloudPtr PandarXTMDecoder::convert_dual(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);
  for (int i = 0; i < packet_.header.chLaserNumber; ++i) {
    for (int block = block_id; block <= block_id + 1; ++block) {
      CalcXTPointXYZIT(block, i, block_pc);
    }
  }

  return block_pc;
}

bool PandarXTMDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE) {
    std::cout << "pandar_packet.size != PACKET_SIZE" << std::endl;
    return false;
  }
  const uint8_t * buf = &pandar_packet.data[0];

  size_t index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
  packet_.header.chLaserNumber = buf[index + 6] & 0xff;
  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
  packet_.header.chReturnType = buf[index + 8] & 0xff;
  packet_.header.chDisUnit = buf[index + 9] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.sob != 0xEEFF) {
    // Error Start of Packet!
    std::cout << "Error Start of Packet!" << std::endl;
    return false;
  }

  for (int8_t block = 0; block < packet_.header.chBlockNumber; block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    index += BLOCK_HEADER_AZIMUTH;

    for (int unit = 0; unit < packet_.header.chLaserNumber; unit++) {
      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      packet_.blocks[block].units[unit].distance =
        (static_cast<float>(unRange * packet_.header.chDisUnit)) / 1000.f;
      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
      packet_.blocks[block].units[unit].confidence = (buf[index + 3] & 0xff);
      index += UNIT_SIZE;
    }
  }

  index += RESERVED_SIZE;  // skip reserved bytes
  packet_.return_mode = buf[index] & 0xff;

  index += RETURN_SIZE;
  index += ENGINE_VELOCITY;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;
  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }

  index += UTC_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;
  index += FACTORY_SIZE;

  return true;
}
}  // namespace pandar_xtm
}  // namespace drivers
}  // namespace nebula
