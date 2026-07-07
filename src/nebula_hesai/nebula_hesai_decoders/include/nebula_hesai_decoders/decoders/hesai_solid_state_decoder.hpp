// Copyright 2026 TIER IV, Inc.
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

#include "nebula_core_decoders/point_filters/blockage_mask.hpp"
#include "nebula_core_decoders/point_filters/downsample_mask.hpp"
#include "nebula_hesai_decoders/decoders/angle_corrector.hpp"
#include "nebula_hesai_decoders/decoders/functional_safety.hpp"
#include "nebula_hesai_decoders/decoders/hesai_packet.hpp"
#include "nebula_hesai_decoders/decoders/hesai_scan_decoder.hpp"
#include "nebula_hesai_decoders/decoders/packet_loss_detector.hpp"

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/stopwatch.hpp>
#include <nebula_hesai_common/hesai_common.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace nebula::drivers
{

template <typename SensorT>
class HesaiSolidStateDecoder : public HesaiScanDecoder  // for Solid State sensor
{
private:
  static constexpr int kRetroMiniFlashRows = 6;
  static constexpr int kRetroMiniFlashCols = 16;
  static constexpr float kRetroCloseDistanceThreshold = 0.05F;
  static constexpr float kRetroCoghThreshold = 280.0F;
  static constexpr float kRetroEnergyThreshold = 26.0F;
  static constexpr float kRetroParamA = 0.85F;
  static constexpr float kRetroParamB = 0.008F;
  static constexpr float kRetroConfidenceThreshold = 96.0F;
  static constexpr float kRetroConfidence = 255.0F;
  static constexpr float kRetroNeighborConfidence = 100.0F;
  static constexpr float kRetroCloseConfidence = 8.0F;
  static constexpr float kRetroZeroConfidence = 0.0F;
  static constexpr size_t kNoPointIndex = std::numeric_limits<size_t>::max();

  static constexpr std::array<double, 512> kRetroCurve{
    0.0, 0.06854, 0.1371, 0.2057, 0.2742, 0.3428, 0.4114, 0.48, 0.5483,
    0.665, 0.793, 0.9204, 1.049, 1.176, 1.293, 1.388, 1.481, 1.576, 1.67,
    1.765, 1.859, 1.943, 2.004, 2.062, 2.123, 2.182, 2.242, 2.3, 2.361, 2.42,
    2.48, 2.54, 2.582, 2.615, 2.65, 2.684, 2.719, 2.752, 2.787, 2.82, 2.855,
    2.889, 2.924, 2.957, 2.992, 3.025, 3.06, 3.094, 3.129, 3.162, 3.197, 3.23,
    3.266, 3.299, 3.334, 3.367, 3.402, 3.436, 3.47, 3.508, 3.547, 3.584, 3.623,
    3.66, 3.7, 3.736, 3.773, 3.812, 3.85, 3.889, 3.926, 3.963, 3.988, 4.016,
    4.043, 4.07, 4.094, 4.12, 4.15, 4.176, 4.203, 4.227, 4.254, 4.28, 4.31,
    4.336, 4.36, 4.383, 4.41, 4.434, 4.46, 4.484, 4.508, 4.535, 4.56, 4.582,
    4.61, 4.633, 4.656, 4.684, 4.71, 4.74, 4.77, 4.8, 4.83, 4.86, 4.89, 4.918,
    4.95, 4.98, 5.008, 5.04, 5.066, 5.094, 5.12, 5.145, 5.17, 5.2, 5.223, 5.25,
    5.273, 5.3, 5.324, 5.35, 5.375, 5.395, 5.418, 5.44, 5.465, 5.484, 5.508,
    5.53, 5.555, 5.574, 5.598, 5.625, 5.652, 5.68, 5.707, 5.734, 5.76, 5.79,
    5.816, 5.844, 5.87, 5.9, 5.926, 5.953, 5.98, 6.0, 6.02, 6.04, 6.06, 6.08,
    6.098, 6.117, 6.137, 6.156, 6.176, 6.195, 6.215, 6.234, 6.254, 6.273, 6.293,
    6.316, 6.336, 6.36, 6.38, 6.402, 6.426, 6.445, 6.47, 6.49, 6.51, 6.53, 6.555,
    6.58, 6.605, 6.633, 6.656, 6.684, 6.707, 6.734, 6.758, 6.785, 6.81, 6.836,
    6.86, 6.887, 6.914, 6.94, 6.97, 6.996, 7.027, 7.055, 7.082, 7.11, 7.137,
    7.168, 7.195, 7.223, 7.25, 7.277, 7.3, 7.324, 7.348, 7.37, 7.39, 7.414, 7.438,
    7.457, 7.48, 7.504, 7.527, 7.547, 7.57, 7.594, 7.63, 7.668, 7.71, 7.75, 7.793,
    7.832, 7.875, 7.914, 7.957, 7.992, 8.016, 8.04, 8.06, 8.08, 8.1, 8.125, 8.15,
    8.17, 8.195, 8.22, 8.234, 8.26, 8.29, 8.31, 8.336, 8.37, 8.39, 8.42, 8.445,
    8.47, 8.5, 8.52, 8.555, 8.58, 8.6, 8.63, 8.66, 8.695, 8.73, 8.76, 8.8, 8.83,
    8.87, 8.9, 8.93, 8.97, 9.0, 9.04, 9.07, 9.1, 9.15, 9.19, 9.23, 9.27, 9.31,
    9.35, 9.4, 9.44, 9.48, 9.52, 9.56, 9.61, 9.664, 9.734, 9.81, 9.89, 9.97, 10.05,
    10.12, 10.195, 10.28, 10.39, 10.5, 10.61, 10.72, 10.83, 10.94, 11.05, 11.16,
    11.29, 11.45, 11.62, 11.78, 11.94, 12.1, 12.28, 12.5, 12.72, 12.94, 13.16,
    13.37, 13.5, 13.63, 13.766, 13.91, 14.04, 14.17, 14.3, 14.43, 14.56, 14.69,
    14.82, 14.945, 15.08, 15.2, 15.336, 15.46, 15.56, 15.61, 15.65, 15.695, 15.734,
    15.78, 15.83, 15.87, 15.914, 15.95, 16.0, 16.05, 16.08, 16.1, 16.11, 16.12,
    16.14, 16.16, 16.17, 16.19, 16.2, 16.22, 16.23, 16.25, 16.27, 16.28, 16.31,
    16.33, 16.34, 16.36, 16.38, 16.39, 16.4, 16.4, 16.42, 16.44, 16.45, 16.45,
    16.47, 16.48, 16.5, 16.52, 16.52, 16.53, 16.55, 16.56, 16.56, 16.58, 16.6,
    16.61, 16.62, 16.62, 16.64, 16.66, 16.67, 16.67, 16.69, 16.7, 16.72, 16.72,
    16.73, 16.75, 16.77, 16.77, 16.78, 16.8, 16.8, 16.81, 16.83, 16.84, 16.84,
    16.86, 16.88, 16.89, 16.89, 16.9, 16.92, 16.94, 16.94, 16.95, 16.97, 16.98,
    16.98, 17.0, 17.02, 17.02, 17.03, 17.05, 17.06, 17.06, 17.08, 17.1, 17.11,
    17.11, 17.12, 17.14, 17.16, 17.16, 17.17, 17.19, 17.19, 17.2, 17.22, 17.23,
    17.23, 17.25, 17.27, 17.28, 17.28, 17.3, 17.31, 17.33, 17.33, 17.34, 17.36,
    17.38, 17.38, 17.39, 17.4, 17.4, 17.42, 17.44, 17.45, 17.45, 17.47, 17.48,
    17.5, 17.5, 17.52, 17.53, 17.55, 17.55, 17.56, 17.58, 17.58, 17.6, 17.61,
    17.62, 17.62, 17.64, 17.66, 17.67, 17.67, 17.69, 17.7, 17.72, 17.72, 17.73,
    17.75, 17.77, 17.77, 17.78, 17.8, 17.8, 17.81, 17.83, 17.84, 17.84, 17.86,
    17.88, 17.89, 17.89, 17.9, 17.92, 17.94, 17.94, 17.95, 17.97, 17.98, 17.98,
    18.0, 18.02, 18.02, 18.03, 18.05, 18.06, 18.06, 18.08, 18.1, 18.11, 18.11,
    18.12, 18.14, 18.16, 18.16, 18.17, 18.19};

  struct RetroFilterPoint
  {
    bool valid{false};
    bool is_retro{false};
    float distance{0.0F};
    float cogh{0.0F};
    float reflectivity{0.0F};
    float confidence{0.0F};
    float energy{0.0F};
    size_t point_index{kNoPointIndex};
  };

  struct ScanCutAngles
  {
    float fov_min;
    float fov_max;
    float scan_emit_angle;
  };

  struct DecodeFrame
  {
    NebulaPointCloudPtr pointcloud;
    uint64_t scan_timestamp_ns{0};
    std::optional<point_filters::BlockageMask> blockage_mask;
    std::vector<RetroFilterPoint> retro_points;
    std::vector<size_t> retro_offsets;
  };

  /// @brief Configuration for this decoder
  const std::shared_ptr<const drivers::HesaiSensorConfiguration> sensor_configuration_;

  /// @brief The sensor definition, used for return mode and time offset handling
  SensorT sensor_{};

  /// @brief A function that is called on each decoded pointcloud frame
  pointcloud_callback_t pointcloud_callback_;

  /// @brief Decodes azimuth/elevation angles given calibration/correction data
  typename SensorT::angle_corrector_t angle_corrector_;

  /// @brief Decodes functional safety data for supported sensors
  std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>>
    functional_safety_decoder_;

  std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> packet_loss_detector_;

  /// @brief The last decoded packet
  typename SensorT::packet_t packet_;

  ScanCutAngles scan_cut_angles_;
  uint32_t last_frame_id_ = 0;

  std::vector<float> retro_distance_ratio_map_;

  std::shared_ptr<loggers::Logger> logger_;

  std::optional<point_filters::DownsampleMaskFilter> mask_filter_;

  std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin_;

  /// @brief Decoded data of the frame currently being decoded to
  DecodeFrame decode_frame_;
  /// @brief Decoded data of the frame currently being output
  DecodeFrame output_frame_;

  /// @brief Validates and parse PandarPacket. Checks size and, if present, CRC checksums.
  /// @param packet The incoming PandarPacket
  /// @return Whether the packet was parsed successfully
  bool parse_packet(const std::vector<uint8_t> & packet)
  {
    if (packet.size() < sizeof(typename SensorT::packet_t)) {
      NEBULA_LOG_STREAM(
        logger_->error, "Packet size mismatch: " << packet.size() << " | Expected at least: "
                                                 << sizeof(typename SensorT::packet_t));
      return false;
    }

    if (!std::memcpy(&packet_, packet.data(), sizeof(typename SensorT::packet_t))) {
      logger_->error("Packet memcopy failed");
      return false;
    }

    return true;
  }

  static std::vector<float> make_retro_distance_ratio_map()
  {
    const int row_count = SensorT::row_N * 2;
    const int col_count = SensorT::col_N * 2;
    std::vector<float> distance_ratio_map(static_cast<size_t>(row_count * col_count), 0.0F);

    for (int row = 0; row < row_count; ++row) {
      for (int col = 0; col < col_count; ++col) {
        const float row_delta = static_cast<float>(row - SensorT::row_N);
        const float col_delta = static_cast<float>(col - SensorT::col_N);
        distance_ratio_map[static_cast<size_t>(row * col_count + col)] =
          static_cast<float>(std::pow(row_delta * row_delta + col_delta * col_delta, kRetroParamA));
      }
    }

    return distance_ratio_map;
  }

  static float compute_retro_energy(const float cogh)
  {
    if (cogh <= 0.0F) {
      return 0.0F;
    }

    const auto pos = static_cast<size_t>(cogh);
    if (pos + 1 >= kRetroCurve.size()) {
      return static_cast<float>(kRetroCurve.back());
    }

    const double cogh_left = static_cast<double>(pos);
    const double cogh_right = static_cast<double>(pos + 1);
    const double energy_left = kRetroCurve[pos];
    const double energy_right = kRetroCurve[pos + 1];
    return static_cast<float>(
      (energy_right - energy_left) / (cogh_right - cogh_left) *
        (static_cast<double>(cogh) - cogh_left) +
      energy_left);
  }

  static uint16_t get_retro_cogh(
    const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return static_cast<uint16_t>((unit.reserved_or_confidence2 & 0x3FU) * 8U) +
           static_cast<uint16_t>((unit.reserved_or_confidence1 & 0xE0U) >> 5U);
  }

  float get_retro_distance_ratio(const int row_delta, const int col_delta) const
  {
    const int col_count = SensorT::col_N * 2;
    const int row = row_delta + SensorT::row_N;
    const int col = col_delta + SensorT::col_N;
    return retro_distance_ratio_map_[static_cast<size_t>(row * col_count + col)];
  }

  void record_retro_blooming_candidate(
    DecodeFrame & frame, const uint32_t row, const uint32_t col,
    const typename SensorT::packet_t::body_t::block_t::unit_t & unit, const float distance,
    const size_t point_index)
  {
    const size_t offset = static_cast<size_t>(row * SensorT::col_N + col);
    auto & candidate = frame.retro_points[offset];
    candidate = RetroFilterPoint{};
    candidate.valid = true;
    candidate.distance = distance;
    candidate.cogh = static_cast<float>(get_retro_cogh(unit));
    candidate.reflectivity = static_cast<float>(unit.reflectivity);
    candidate.confidence = packet_.header.first_block_return > 0
                             ? 254.0F + 86.0F -
                                 static_cast<float>(packet_.header.first_block_return) * 86.0F
                             : 254.0F;
    candidate.point_index = point_index;

    if (candidate.distance <= 0.0F || candidate.cogh <= 0.0F) {
      candidate.confidence = kRetroZeroConfidence;
    } else if (candidate.distance < kRetroCloseDistanceThreshold) {
      candidate.confidence = kRetroCloseConfidence;
    }

    if (candidate.confidence < kRetroConfidenceThreshold) {
      return;
    }

    const float log2_energy = compute_retro_energy(candidate.cogh);
    candidate.energy = std::pow(2.0F, log2_energy);

    if (
      candidate.confidence == kRetroConfidence || candidate.cogh >= kRetroCoghThreshold ||
      log2_energy + 2.0F * std::log2(candidate.distance / 0.15F) >=
        kRetroEnergyThreshold ||
      (candidate.distance > 10.0F && candidate.cogh > kRetroCoghThreshold * 0.5F &&
       candidate.reflectivity >= 200.0F)) {
      candidate.confidence = kRetroConfidence;
      candidate.is_retro = true;
      frame.retro_offsets.push_back(offset);
    }
  }

  void apply_retro_blooming_filter(DecodeFrame & frame)
  {
    if (frame.retro_offsets.empty() || frame.pointcloud->empty()) {
      return;
    }

    for (const size_t retro_offset : frame.retro_offsets) {
      auto & retro = frame.retro_points[retro_offset];
      if (!retro.valid || !retro.is_retro) {
        continue;
      }

      const int retro_row = static_cast<int>(retro_offset / SensorT::col_N);
      const int retro_col = static_cast<int>(retro_offset % SensorT::col_N);
      const int miniflash_row = retro_row / kRetroMiniFlashRows;
      const int miniflash_col = retro_col / kRetroMiniFlashCols;
      const int min_row = std::max(kRetroMiniFlashRows * miniflash_row - kRetroMiniFlashRows, 0);
      const int max_row =
        std::min(kRetroMiniFlashRows * (miniflash_row + 1) + kRetroMiniFlashRows, SensorT::row_N);
      const int min_col = std::max(kRetroMiniFlashCols * miniflash_col - kRetroMiniFlashCols, 0);
      const int max_col =
        std::min(kRetroMiniFlashCols * (miniflash_col + 1) + kRetroMiniFlashCols, SensorT::col_N);

      for (int row = min_row; row < max_row; ++row) {
        for (int col = min_col; col < max_col; ++col) {
          if (row == retro_row && col == retro_col) {
            continue;
          }

          auto & candidate =
            frame.retro_points[static_cast<size_t>(row * SensorT::col_N + col)];
          if (
            !candidate.valid || candidate.is_retro ||
            candidate.confidence < kRetroConfidenceThreshold ||
            std::abs(candidate.distance - retro.distance) >= 0.5F) {
            continue;
          }

          const float distance_delta = retro.distance - candidate.distance;
          const float cogh_distance_delta = (retro.cogh - candidate.cogh) / 600.0F;
          if (
            std::abs(distance_delta) >= 0.3F &&
            std::abs(distance_delta - cogh_distance_delta) >= 0.3F) {
            continue;
          }

          const float distance_ratio = get_retro_distance_ratio(retro_row - row, retro_col - col);
          if (distance_ratio <= 0.0F) {
            continue;
          }

          candidate.energy -= retro.energy / distance_ratio * kRetroParamB;
          candidate.confidence = kRetroNeighborConfidence;
        }
      }
    }

    std::vector<bool> remove_point(frame.pointcloud->size(), false);
    bool did_mark_point = false;
    for (const auto & candidate : frame.retro_points) {
      if (
        candidate.valid && candidate.point_index != kNoPointIndex &&
        candidate.point_index < remove_point.size() && candidate.energy < 0.0F &&
        candidate.distance > 1.0F) {
        remove_point[candidate.point_index] = true;
        did_mark_point = true;
      }
    }

    if (!did_mark_point) {
      return;
    }

    size_t write_index = 0;
    for (size_t read_index = 0; read_index < frame.pointcloud->size(); ++read_index) {
      if (!remove_point[read_index]) {
        (*frame.pointcloud)[write_index] = (*frame.pointcloud)[read_index];
        ++write_index;
      }
    }
    frame.pointcloud->resize(write_index);
  }

  static void reset_retro_blooming_filter(DecodeFrame & frame)
  {
    std::fill(frame.retro_points.begin(), frame.retro_points.end(), RetroFilterPoint{});
    frame.retro_offsets.clear();
  }

  /// @brief Converts each channel in the packet to a NebulaPoint and appends it to the point cloud
  /// @param start_block_id Unused for FTX (always 0)
  /// @param n_returns Unused for FTX (always single-return)
  void convert_returns(size_t start_block_id, size_t /*n_returns*/)
  {
    (void)start_block_id;

    uint64_t packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    
    // For FT series (FTX140, FTX180, etc.), the packet tail contains row_id and column_id
    // which represent the "packet row" and "packet col" (pk_row, pk_col).
    const uint32_t pk_row = packet_.tail.row_id;
    const uint32_t pk_col = packet_.tail.column_id;

    // If the blockage mask plugin is not present, we can return early if distance checks fail
    const bool filters_can_return_early = !blockage_mask_plugin_;

    std::vector<const typename SensorT::packet_t::body_t::block_t::unit_t *> dummy_return_units;
    const auto return_type = sensor_.get_return_type(
      static_cast<hesai_packet::return_mode::ReturnMode>(packet_.tail.return_mode),
      1 /* single return for FTX */, dummy_return_units);

    for (size_t ch = 0; ch < SensorT::packet_t::n_channels; ++ch) {
      // Calculate true pixel row and column from the 1-indexed channel id (as in the manual):
      //   ch_row = pk_row * 6 + (ch - 1) % 6
      //   ch_col = pk_col * 16 + (ch - 1) / 6
      // ch is 0-indexed in code, so ch_1 = ch + 1 is the 1-indexed channel.
      const uint32_t ch_1 = ch + 1;
      const uint32_t ch_row = pk_row * 6 + (ch_1 - 1) % 6;
      const uint32_t ch_col = pk_col * 16 + (ch_1 - 1) / 6;

      // Bounds check against sensor's maximum dimensions
      if (ch_row >= SensorT::row_N || ch_col >= SensorT::col_N) {
        continue;
      }

      auto & unit = packet_.body.blocks[0].units[ch];

      const CorrectedAngleData corrected_angle_data =
        angle_corrector_.get_corrected_angle_data(ch_row, ch_col);

      bool point_is_valid = true;

      if (unit.distance == 0) {
        point_is_valid = false;
      }

      float distance = get_distance(unit);

      if (
        distance < SensorT::min_range || SensorT::max_range < distance ||
        distance < sensor_configuration_->min_range ||
        sensor_configuration_->max_range < distance) {
        point_is_valid = false;
      }

      if (filters_can_return_early && !point_is_valid) {
        continue;
      }

      float azimuth = corrected_angle_data.azimuth_rad;

      const float max_angle = static_cast<float>(2. * M_PI);
      const float azimuth_norm = normalize_angle(azimuth, max_angle);
      const float fov_min_norm = normalize_angle(scan_cut_angles_.fov_min, max_angle);
      const float fov_max_norm = normalize_angle(scan_cut_angles_.fov_max, max_angle);

      const bool in_fov = angle_is_between(fov_min_norm, fov_max_norm, azimuth_norm);
      if (!in_fov) {
        continue;
      }

      bool in_current_scan = true;

      auto & frame = in_current_scan ? decode_frame_ : output_frame_;

      if (frame.blockage_mask) {
        frame.blockage_mask->update(azimuth, ch, sensor_.get_blockage_type(unit.distance));
      }

      if (!point_is_valid) {
        continue;
      }

      NebulaPoint point;
      point.distance = distance;
      point.intensity = unit.reflectivity;
      point.time_stamp = packet_timestamp_ns - frame.scan_timestamp_ns;

      point.return_type = static_cast<uint8_t>(return_type);
      point.channel = ch;

      // Use sin/cos functions from calibration data from corrected_angle_data
      const float xy_distance = distance * corrected_angle_data.cos_elevation;
      point.x = xy_distance * corrected_angle_data.sin_azimuth;
      point.y = xy_distance * corrected_angle_data.cos_azimuth;
      point.z = distance * corrected_angle_data.sin_elevation;

      // The driver wrapper converts to degrees, expects radians
      point.azimuth = corrected_angle_data.azimuth_rad;
      point.elevation = corrected_angle_data.elevation_rad;

      const bool is_masked = mask_filter_ && mask_filter_->excluded(point);
      record_retro_blooming_candidate(
        frame, ch_row, ch_col, unit, distance, is_masked ? kNoPointIndex : frame.pointcloud->size());

      if (!is_masked) {
        frame.pointcloud->emplace_back(point);
      }
    }
  }

  /// @brief Get the distance of the given unit in meters
  float get_distance(const typename SensorT::packet_t::body_t::block_t::unit_t & unit)
  {
    return unit.distance * (static_cast<double>(packet_.header.dis_unit) / 1000.0);
  }

  /// @brief Get timestamp of point in nanoseconds, relative to scan timestamp. Includes firing time
  /// offset correction for channel and block
  /// @param scan_timestamp_ns Start timestamp of the current scan in nanoseconds
  /// @param packet_timestamp_ns The timestamp of the current PandarPacket in nanoseconds
  /// @param block_id The block index of the point
  /// @param channel_id The channel index of the point
  uint32_t get_point_time_relative(
    uint64_t scan_timestamp_ns, uint64_t packet_timestamp_ns, size_t block_id, size_t channel_id)
  {
    (void)block_id;
    (void)channel_id;

    // this is a flash solid state LIDAR, point_to_packet_offset_ns is 0 as measurements comes from
    // the same light emission and there is non need to correct packet_to_scan_offset_ns
    auto packet_to_scan_offset_ns = static_cast<uint32_t>(packet_timestamp_ns - scan_timestamp_ns);
    return packet_to_scan_offset_ns;
  }

  DecodeFrame initialize_frame() const
  {
    DecodeFrame frame;
    frame.pointcloud = std::make_shared<NebulaPointCloud>();
    frame.pointcloud->reserve(SensorT::max_scan_buffer_points);
    frame.retro_points.resize(static_cast<size_t>(SensorT::row_N * SensorT::col_N));

    if (blockage_mask_plugin_) {
      frame.blockage_mask = point_filters::BlockageMask(
        SensorT::fov_mdeg.azimuth, blockage_mask_plugin_->get_bin_width_mdeg(),
        SensorT::packet_t::n_channels);
    }

    return frame;
  }

  /// @brief Called when a scan is complete, published and then clears the output frame.
  void on_scan_complete()
  {
    double scan_timestamp_s = static_cast<double>(output_frame_.scan_timestamp_ns) * 1e-9;

    if (pointcloud_callback_) {
      pointcloud_callback_(output_frame_.pointcloud, scan_timestamp_s);
    }

    if (blockage_mask_plugin_ && output_frame_.blockage_mask) {
      blockage_mask_plugin_->callback_and_reset(
        output_frame_.blockage_mask.value(), scan_timestamp_s);
    }

    output_frame_.pointcloud->clear();
    reset_retro_blooming_filter(output_frame_);
  }

public:
  /// @brief Constructor
  /// @param sensor_configuration SensorConfiguration for this decoder
  /// @param correction_data Calibration data for this decoder
  explicit HesaiSolidStateDecoder(
    const std::shared_ptr<const HesaiSensorConfiguration> & sensor_configuration,
    const std::shared_ptr<const typename SensorT::angle_corrector_t::correction_data_t> &
      correction_data,
    const std::shared_ptr<loggers::Logger> & logger,
    const std::shared_ptr<FunctionalSafetyDecoderTypedBase<typename SensorT::packet_t>> &
      functional_safety_decoder,
    const std::shared_ptr<PacketLossDetectorTypedBase<typename SensorT::packet_t>> &
      packet_loss_detector,
    std::shared_ptr<point_filters::BlockageMaskPlugin> blockage_mask_plugin)
  : sensor_configuration_(sensor_configuration),
    angle_corrector_(correction_data),
    functional_safety_decoder_(functional_safety_decoder),
    packet_loss_detector_(packet_loss_detector),
    scan_cut_angles_(
      {static_cast<float>(deg2rad(sensor_configuration_->cloud_min_angle)),
       static_cast<float>(deg2rad(sensor_configuration_->cloud_max_angle)),
       static_cast<float>(deg2rad(sensor_configuration_->cut_angle))}),
    retro_distance_ratio_map_(make_retro_distance_ratio_map()),
    logger_(logger),
    blockage_mask_plugin_(std::move(blockage_mask_plugin)),
    decode_frame_(initialize_frame()),
    output_frame_(initialize_frame())
  {
    if (sensor_configuration->downsample_mask_path) {
      mask_filter_ = point_filters::DownsampleMaskFilter(
        sensor_configuration->downsample_mask_path.value(), SensorT::fov_mdeg.azimuth,
        SensorT::peak_resolution_mdeg.azimuth, SensorT::packet_t::n_channels,
        logger_->child("Downsample Mask"), true, sensor_.get_dither_transform());
    }
  }

  void set_pointcloud_callback(pointcloud_callback_t callback) override
  {
    pointcloud_callback_ = std::move(callback);
  }

  PacketDecodeResult unpack(const std::vector<uint8_t> & packet) override
  {
    util::Stopwatch decode_watch;

    if (!parse_packet(packet)) {
      return {PerformanceCounters{decode_watch.elapsed_ns()}, DecodeError::PACKET_PARSE_FAILED};
    }
    if (packet_loss_detector_) {
      packet_loss_detector_->update(packet_);
    }

    // Even if the checksums of other parts of the packet are invalid, functional safety info
    // is still checked. This is a null-op for sensors that do not support functional safety.
    if (functional_safety_decoder_) {
      functional_safety_decoder_->update(packet_);
    }

    // FYI: This is where the CRC would be checked. Since this caused performance issues in the
    // past, and since the frame check sequence of the packet is already checked by the NIC, we skip
    // it here.

    // This is the first scan, set scan timestamp to whatever packet arrived first
    // It is valid for a flash LIDAR sensor as the FT120
    if (decode_frame_.scan_timestamp_ns == 0) {
      decode_frame_.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
      last_frame_id_ = packet_.tail.frame_id;
    }

    bool did_scan_complete = false;
    const auto current_frame_id = packet_.tail.frame_id;

    // We have a new scan when frame_id changes
    if (last_frame_id_ != current_frame_id && last_frame_id_ != 0) {
      // Swapping decode_frame_ to output_frame_ so it can be published
      std::swap(decode_frame_, output_frame_);
      did_scan_complete = true;

      // The new scan starts with this packet
      decode_frame_.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
      decode_frame_.pointcloud->clear();
      reset_retro_blooming_filter(decode_frame_);
    }

    if (decode_frame_.scan_timestamp_ns == 0) {
      decode_frame_.scan_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    }

    convert_returns(0, 1);

    last_frame_id_ = current_frame_id;

    uint64_t decode_duration_ns = decode_watch.elapsed_ns();
    uint64_t callbacks_duration_ns = 0;

    if (did_scan_complete) {
      apply_retro_blooming_filter(output_frame_);
      util::Stopwatch callback_watch;
      on_scan_complete();
      callbacks_duration_ns += callback_watch.elapsed_ns();
    }

    PacketMetadata metadata;
    metadata.packet_timestamp_ns = hesai_packet::get_timestamp_ns(packet_);
    metadata.did_scan_complete = did_scan_complete;
    return {PerformanceCounters{decode_duration_ns - callbacks_duration_ns}, metadata};
  }
};

}  // namespace nebula::drivers
