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

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_core_ros/point_cloud_conversions.hpp>
#include <nebula_core_ros/rclcpp_logger.hpp>
#include <nebula_hesai_common/hesai_common.hpp>
#include <nebula_hesai_decoders/hesai_driver.hpp>
#include <nebula_robosense_common/robosense_common.hpp>
#include <nebula_robosense_decoders/robosense_driver.hpp>
#include <nebula_robosense_decoders/robosense_info_driver.hpp>
#include <nebula_seyond_common/seyond_calibration_data.hpp>
#include <nebula_seyond_common/seyond_common.hpp>
#include <nebula_seyond_common/seyond_configuration.hpp>
#include <nebula_seyond_decoders/seyond_decoder.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tins/tins.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
namespace fs = std::filesystem;

constexpr double kUnfilteredMaxRangeMeters = 1000.0;
constexpr uint64_t kMaxRosHeaderTimeNs =
  static_cast<uint64_t>(std::numeric_limits<int32_t>::max()) * 1000000000ULL + 999999999ULL;

builtin_interfaces::msg::Time to_ros_stamp(uint64_t timestamp_ns)
{
  const uint64_t clamped = std::min(timestamp_ns, kMaxRosHeaderTimeNs);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(clamped / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(clamped % 1000000000ULL);
  return stamp;
}

rclcpp::Time to_rclcpp_time(uint64_t timestamp_ns)
{
  const uint64_t clamped =
    std::min<uint64_t>(timestamp_ns, static_cast<uint64_t>(std::numeric_limits<int64_t>::max()));
  return rclcpp::Time(static_cast<int64_t>(clamped));
}

uint64_t seconds_to_ns(double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return 0;
  }
  const long double ns = static_cast<long double>(seconds) * 1000000000.0L;
  return static_cast<uint64_t>(
    std::min<long double>(ns, static_cast<long double>(std::numeric_limits<uint64_t>::max())));
}

struct UdpPayload
{
  uint64_t timestamp_ns{};
  std::vector<uint8_t> payload;
};

uint64_t timestamp_to_ns(const Tins::Timestamp & timestamp)
{
  return static_cast<uint64_t>(timestamp.seconds()) * 1000000000ULL +
         static_cast<uint64_t>(timestamp.microseconds()) * 1000ULL;
}

class PcapReader
{
public:
  explicit PcapReader(const fs::path & path) : sniffer_(path.string(), "ip") {}

  bool next_udp_payload(UdpPayload & out)
  {
    while (auto packet = sniffer_.next_packet()) {
      auto * pdu = packet.pdu();
      if (pdu == nullptr) {
        continue;
      }

      const auto reassembly_status = reassembler_.process(*pdu);
      if (reassembly_status == Tins::IPv4Reassembler::FRAGMENTED) {
        continue;
      }

      auto * udp = pdu->find_pdu<Tins::UDP>();
      auto * raw = pdu->find_pdu<Tins::RawPDU>();
      if (udp == nullptr || raw == nullptr) {
        continue;
      }

      out.timestamp_ns = timestamp_to_ns(packet.timestamp());
      out.payload = raw->payload();
      if (!out.payload.empty()) {
        return true;
      }
    }

    return false;
  }

private:
  Tins::FileSniffer sniffer_;
  Tins::IPv4Reassembler reassembler_;
};

enum class Vendor {
  Hesai,
  Robosense,
  Seyond,
};

struct SensorSpec
{
  std::string filename_prefix;
  Vendor vendor;
  std::string model;
  std::string frame_id;
  std::vector<size_t> data_lengths;
  std::vector<uint8_t> data_magic;
  fs::path calibration_relpath;
  double min_range{0.0};
  double max_range{0.0};
  uint16_t cloud_min_angle{0};
  uint16_t cloud_max_angle{360};
  uint16_t sync_angle{0};
  double cut_angle{360.0};
  uint16_t rotation_speed{600};
  std::string return_mode;
};

std::vector<SensorSpec> make_specs()
{
  // clang-format off
  return {
    {"hesai_at128_", Vendor::Hesai, "PandarAT128", "at128", {1118}, {}, "at128/PandarAT128.dat", 0.3, kUnfilteredMaxRangeMeters, 30, 150, 30, 150.0, 200, "Dual"},
    {"hesai_ftx140_", Vendor::Hesai, "FTX140", "ftx140", {554}, {0xee, 0xff}, "hesai_ftx140/ftx140.dat", 0.05, kUnfilteredMaxRangeMeters, 20, 160, 20, 160.0, 600, "First"},
    {"hesai_ftx180_", Vendor::Hesai, "FTX180", "ftx180", {554}, {0xee, 0xff}, "hesai_ftx180/ftx180.dat", 0.05, kUnfilteredMaxRangeMeters, 0, 180, 90, 180.0, 600, "First"},
    {"robosense_e1_", Vendor::Robosense, "E1", "e1", {1200}, {0x55, 0xaa, 0x5a, 0xa5}, {}, 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Strongest"},
    {"robosense_em4_", Vendor::Robosense, "EM4", "em4", {1084}, {0x55, 0xaa, 0x5a, 0xa5}, {}, 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Strongest"},
    {"robosense_emx_", Vendor::Robosense, "EMX", "emx", {812}, {0x55, 0xaa, 0x5a, 0xa5}, {}, 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Strongest"},
    {"seyond_hummingbird_d1_", Vendor::Seyond, "HummingbirdD1", "hummingbirdd1", {}, {0x6a, 0x17}, "seyond_hummingbirdd1/anglehv_table.bin", 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Single"},
    {"seyond_robin_e_", Vendor::Seyond, "RobinE1X", "robine1x", {}, {0x6a, 0x17}, "seyond_robine1x/anglehv_table.bin", 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Single"},
    {"seyond_robin_w_", Vendor::Seyond, "RobinW", "robinw", {}, {0x6a, 0x17}, "seyond_robinw/anglehv_table.bin", 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Single"},
  };
  // clang-format on
}

std::string sensor_name(const SensorSpec & spec)
{
  auto name = spec.filename_prefix;
  if (!name.empty() && name.back() == '_') {
    name.pop_back();
  }
  return name;
}

std::string pointcloud_topic(const SensorSpec & spec)
{
  return "/" + spec.frame_id + "/pointcloud";
}

const SensorSpec * find_spec(const fs::path & pcap, const std::vector<SensorSpec> & specs)
{
  const std::string filename = pcap.filename().string();
  for (const auto & spec : specs) {
    if (filename.rfind(spec.filename_prefix, 0) == 0) {
      return &spec;
    }
  }
  return nullptr;
}

bool has_magic(const std::vector<uint8_t> & payload, const std::vector<uint8_t> & magic)
{
  return magic.empty() || (payload.size() >= magic.size() &&
                           std::equal(magic.begin(), magic.end(), payload.begin()));
}

bool is_data_payload(const SensorSpec & spec, const std::vector<uint8_t> & payload)
{
  const bool length_ok =
    spec.data_lengths.empty() ||
    std::find(spec.data_lengths.begin(), spec.data_lengths.end(), payload.size()) !=
      spec.data_lengths.end();
  return length_ok && has_magic(payload, spec.data_magic);
}

struct ConversionStats
{
  size_t udp_packets{0};
  size_t data_packets{0};
  size_t info_packets{0};
  size_t pointclouds{0};
  size_t points{0};
};

template <typename Callback>
void for_each_udp_payload(const fs::path & pcap, Callback && callback)
{
  PcapReader reader(pcap);
  UdpPayload payload;
  while (reader.next_udp_payload(payload)) {
    callback(payload);
  }
}

class McapPointcloudWriter
{
public:
  McapPointcloudWriter(fs::path output_path, std::string topic, std::string frame_id)
  : output_path_(std::move(output_path)), topic_(std::move(topic)), frame_id_(std::move(frame_id))
  {
    if (fs::exists(output_path_)) {
      fs::remove_all(output_path_);
    }
    if (!output_path_.parent_path().empty()) {
      fs::create_directories(output_path_.parent_path());
    }

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = output_path_.string();
    storage_options.storage_id = "mcap";
    rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
    writer_.open(storage_options, converter_options);
    writer_.create_topic({topic_, "sensor_msgs/msg/PointCloud2", "cdr", ""});
  }

  void write(
    const nebula::drivers::NebulaPointCloudPtr & pointcloud, uint64_t timestamp_ns,
    uint64_t fallback_timestamp_ns)
  {
    if (!pointcloud || pointcloud->empty()) {
      return;
    }

    if (timestamp_ns == 0) {
      timestamp_ns = fallback_timestamp_ns;
    }

    auto msg = nebula::ros::to_ros_msg(*pointcloud);
    msg.header.frame_id = frame_id_;
    msg.header.stamp = to_ros_stamp(timestamp_ns);
    writer_.write(msg, topic_, to_rclcpp_time(timestamp_ns));
    ++clouds_written_;
    points_written_ += pointcloud->size();
  }

  size_t clouds_written() const { return clouds_written_; }
  size_t points_written() const { return points_written_; }

private:
  fs::path output_path_;
  std::string topic_;
  std::string frame_id_;
  rosbag2_cpp::Writer writer_;
  size_t clouds_written_{0};
  size_t points_written_{0};
};

void update_cloud_stats(ConversionStats & stats, const McapPointcloudWriter & writer)
{
  stats.pointclouds = writer.clouds_written();
  stats.points = writer.points_written();
}

ConversionStats decode_hesai(
  const fs::path & pcap, const SensorSpec & spec, const fs::path & calibration_root,
  McapPointcloudWriter & writer)
{
  auto config = std::make_shared<nebula::drivers::HesaiSensorConfiguration>();
  config->sensor_model = nebula::drivers::sensor_model_from_string(spec.model);
  config->return_mode =
    nebula::drivers::return_mode_from_string_hesai(spec.return_mode, config->sensor_model);
  config->host_ip = "0.0.0.0";
  config->sensor_ip = "0.0.0.0";
  config->multicast_ip = "";
  config->data_port = 0;
  config->gnss_port = 0;
  config->udp_socket_receive_buffer_size_bytes = 0;
  config->frame_id = spec.frame_id;
  config->packet_mtu_size = 1500;
  config->min_range = spec.min_range;
  config->max_range = spec.max_range;
  config->use_sensor_time = false;
  config->sync_angle = spec.sync_angle;
  config->cut_angle = spec.cut_angle;
  config->dual_return_distance_threshold = 0.1;
  config->calibration_path = (calibration_root / spec.calibration_relpath).string();
  config->calibration_download_enabled = false;
  config->rotation_speed = spec.rotation_speed;
  config->cloud_min_angle = spec.cloud_min_angle;
  config->cloud_max_angle = spec.cloud_max_angle;
  config->ptp_profile = nebula::drivers::PtpProfile::UNKNOWN_PROFILE;
  config->ptp_domain = 0;
  config->ptp_transport_type = nebula::drivers::PtpTransportType::UNKNOWN_TRANSPORT;
  config->ptp_switch_type = nebula::drivers::PtpSwitchType::UNKNOWN_SWITCH;
  config->ptp_lock_threshold = 0;
  config->hires_mode = false;

  std::shared_ptr<nebula::drivers::HesaiCalibrationConfigurationBase> calibration;
  if (
    config->sensor_model == nebula::drivers::SensorModel::HESAI_FTX140 ||
    config->sensor_model == nebula::drivers::SensorModel::HESAI_FTX180) {
    calibration = std::make_shared<nebula::drivers::HesaiCorrectionFTX>();
  } else {
    calibration = std::make_shared<nebula::drivers::HesaiCorrection>();
  }
  calibration->calibration_file = config->calibration_path;
  if (calibration->load_from_file(config->calibration_path) != nebula::Status::OK) {
    throw std::runtime_error("Failed to load Hesai calibration: " + config->calibration_path);
  }

  uint64_t last_packet_timestamp_ns = 0;
  auto logger = std::make_shared<nebula::drivers::loggers::RclcppLogger>("nebula_jari_tools.hesai");
  nebula::drivers::HesaiDriver driver(
    config, calibration, logger,
    [&](const nebula::drivers::NebulaPointCloudPtr & pointcloud, double timestamp_s) {
      writer.write(pointcloud, seconds_to_ns(timestamp_s), last_packet_timestamp_ns);
    });
  if (driver.get_status() != nebula::Status::OK) {
    throw std::runtime_error("Failed to initialize Hesai decoder for " + spec.model);
  }

  ConversionStats stats;
  for_each_udp_payload(pcap, [&](const UdpPayload & payload) {
    ++stats.udp_packets;
    if (!is_data_payload(spec, payload.payload)) {
      return;
    }
    ++stats.data_packets;
    last_packet_timestamp_ns = payload.timestamp_ns;
    driver.parse_cloud_packet(payload.payload);
    update_cloud_stats(stats, writer);
  });
  return stats;
}

std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> make_robosense_config(
  const SensorSpec & spec)
{
  auto config = std::make_shared<nebula::drivers::RobosenseSensorConfiguration>();
  config->sensor_model = nebula::drivers::sensor_model_from_string(spec.model);
  config->return_mode = nebula::drivers::return_mode_from_string_robosense(spec.return_mode);
  config->host_ip = "0.0.0.0";
  config->sensor_ip = "0.0.0.0";
  config->data_port = 6699;
  config->gnss_port = 7788;
  config->difop2_port = 7788;
  config->frame_id = spec.frame_id;
  config->packet_mtu_size = 1500;
  config->min_range = spec.min_range;
  config->max_range = spec.max_range;
  config->use_sensor_time = false;
  config->scan_phase = 0.0;
  config->dual_return_distance_threshold = 0.1;
  return config;
}

bool directional_calibration_ready(
  nebula::drivers::SensorModel model,
  const nebula::drivers::RobosenseCalibrationConfiguration & calibration)
{
  if (model == nebula::drivers::SensorModel::ROBOSENSE_EMX) {
    return calibration.pixel_pitch.size() == 192 && calibration.surface_pitch_offset.size() == 2;
  }
  if (model == nebula::drivers::SensorModel::ROBOSENSE_EM4) {
    return calibration.pixel_pitch.size() == 520;
  }
  return true;
}

struct RobosenseDecoderSeed
{
  std::shared_ptr<const nebula::drivers::RobosenseSensorConfiguration> config;
  std::shared_ptr<const nebula::drivers::RobosenseCalibrationConfiguration> calibration;
  size_t info_packets{0};
};

ConversionStats decode_robosense(
  const fs::path & pcap, const SensorSpec & spec, const RobosenseDecoderSeed & seed,
  McapPointcloudWriter & writer)
{
  auto driver = std::make_shared<nebula::drivers::RobosenseDriver>(seed.config, seed.calibration);
  if (driver->get_status() != nebula::Status::OK) {
    throw std::runtime_error("Failed to initialize RoboSense decoder for " + spec.model);
  }

  ConversionStats stats;
  stats.info_packets = seed.info_packets;
  for_each_udp_payload(pcap, [&](const UdpPayload & payload) {
    ++stats.udp_packets;
    if (!is_data_payload(spec, payload.payload)) {
      return;
    }
    ++stats.data_packets;
    const auto pointcloud_ts = driver->parse_cloud_packet(payload.payload);
    if (const auto pointcloud = std::get<0>(pointcloud_ts)) {
      writer.write(pointcloud, seconds_to_ns(std::get<1>(pointcloud_ts)), payload.timestamp_ns);
    }
    update_cloud_stats(stats, writer);
  });
  return stats;
}

ConversionStats decode_seyond(
  const fs::path & pcap, const SensorSpec & spec, const fs::path & calibration_root,
  McapPointcloudWriter & writer)
{
  nebula::drivers::SeyondSensorConfiguration config;
  config.sensor_model = nebula::drivers::seyond_sensor_model_from_string(spec.model);
  if (config.sensor_model == nebula::drivers::SeyondSensorModel::UNKNOWN) {
    throw std::runtime_error("Unknown Seyond model: " + spec.model);
  }
  config.connection = {"0.0.0.0", "0.0.0.0", "", "", 0, 0, 0};
  config.use_sensor_time = true;
  config.frame_id = spec.frame_id;
  config.setup_sensor = false;
  config.return_mode = nebula::drivers::return_mode_from_string_seyond(spec.return_mode);
  config.reflectance_mode = nebula::drivers::SeyondReflectanceMode::REFLECTIVITY;
  config.sync_mode = nebula::drivers::SeyondSyncMode::HOST;
  config.frame_rate = 0.0;
  config.horizontal_roi = 10000.0;
  config.vertical_roi = 10000.0;

  const auto calibration_path = calibration_root / spec.calibration_relpath;
  auto calibration_result =
    nebula::drivers::SeyondCalibrationData::load_from_file(calibration_path.string());
  if (!calibration_result.has_value()) {
    throw std::runtime_error("Failed to load Seyond calibration: " + calibration_path.string());
  }

  uint64_t last_packet_timestamp_ns = 0;
  nebula::drivers::SeyondDecoder decoder(
    config,
    [&](nebula::drivers::NebulaPointCloudPtr pointcloud, uint64_t base_timestamp_ns) {
      const uint64_t cloud_timestamp_ns =
        base_timestamp_ns +
        (pointcloud && !pointcloud->empty() ? pointcloud->front().time_stamp : 0);
      writer.write(pointcloud, cloud_timestamp_ns, last_packet_timestamp_ns);
    },
    calibration_result.value());

  ConversionStats stats;
  for_each_udp_payload(pcap, [&](const UdpPayload & payload) {
    ++stats.udp_packets;
    if (!is_data_payload(spec, payload.payload)) {
      return;
    }
    ++stats.data_packets;
    last_packet_timestamp_ns = payload.timestamp_ns;
    decoder.unpack(payload.payload);
    update_cloud_stats(stats, writer);
  });
  return stats;
}

struct Options
{
  fs::path input_pcap;
  fs::path output_mcap;
  std::optional<fs::path> calibration_root;
};

void print_usage(const char * argv0)
{
  std::cerr << "Usage: " << argv0 << " INPUT_PCAP OUTPUT_MCAP [--calibration-root PATH]\n";
}

Options parse_args(int argc, char ** argv)
{
  Options options;
  std::vector<fs::path> positional;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto need_value = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--calibration-root") {
      options.calibration_root = fs::path(need_value(arg));
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (!arg.empty() && arg.front() == '-') {
      throw std::runtime_error("Unknown argument: " + arg);
    } else {
      positional.emplace_back(arg);
    }
  }

  if (positional.size() != 2) {
    throw std::runtime_error("Expected INPUT_PCAP and OUTPUT_MCAP");
  }

  options.input_pcap = positional[0];
  options.output_mcap = positional[1];
  return options;
}

fs::path infer_calibration_root(
  const fs::path & pcap, const SensorSpec & spec, const std::optional<fs::path> & override_root)
{
  if (override_root) {
    return *override_root;
  }

  if (spec.calibration_relpath.empty()) {
    return {};
  }

  for (auto dir = fs::absolute(pcap).parent_path(); !dir.empty(); dir = dir.parent_path()) {
    const auto candidate = dir / "calibration_files";
    if (fs::exists(candidate / spec.calibration_relpath)) {
      return candidate;
    }
    if (dir == dir.root_path()) {
      break;
    }
  }

  throw std::runtime_error(
    "Could not infer calibration root for " + sensor_name(spec) + "; pass --calibration-root PATH");
}

std::optional<RobosenseDecoderSeed> try_preload_robosense_difop(
  const fs::path & pcap, const fs::path & difop_source_pcap, const SensorSpec & spec)
{
  auto base_config = make_robosense_config(spec);
  nebula::drivers::RobosenseInfoDriver info_driver(base_config);
  PcapReader reader(difop_source_pcap);
  UdpPayload payload;
  size_t scanned_udp_packets = 0;
  size_t info_packets = 0;

  while (reader.next_udp_payload(payload)) {
    ++scanned_udp_packets;
    if (info_driver.decode_info_packet(payload.payload) != nebula::Status::OK) {
      continue;
    }
    ++info_packets;

    auto calibration = info_driver.get_sensor_calibration();
    if (!directional_calibration_ready(base_config->sensor_model, calibration)) {
      continue;
    }

    auto config = std::make_shared<nebula::drivers::RobosenseSensorConfiguration>(*base_config);
    const auto sensor_return_mode = info_driver.get_return_mode();
    if (sensor_return_mode != nebula::drivers::ReturnMode::UNKNOWN) {
      config->return_mode = sensor_return_mode;
    }
    config->use_sensor_time = info_driver.get_sync_status();
    calibration.create_corrected_channels();

    std::cout << "PRELOAD " << pcap << " robosense_difop_source=" << difop_source_pcap
              << " robosense_difop_after_udp=" << scanned_udp_packets
              << " decoded_info_packets=" << info_packets << std::endl;
    return RobosenseDecoderSeed{
      config,
      std::make_shared<const nebula::drivers::RobosenseCalibrationConfiguration>(
        std::move(calibration)),
      info_packets};
  }

  return std::nullopt;
}

bool is_same_sensor_pcap(const fs::path & pcap, const SensorSpec & spec)
{
  const auto filename = pcap.filename().string();
  return pcap.extension() == ".pcap" && filename.rfind(spec.filename_prefix, 0) == 0;
}

std::vector<fs::path> candidate_difop_pcaps(const fs::path & pcap, const SensorSpec & spec)
{
  std::vector<fs::path> candidates;
  const auto add_candidate = [&](const fs::path & candidate) {
    std::error_code ec;
    if (
      !fs::is_regular_file(candidate, ec) || !is_same_sensor_pcap(candidate, spec) ||
      std::find(candidates.begin(), candidates.end(), candidate) != candidates.end()) {
      return;
    }
    candidates.push_back(candidate);
  };

  const fs::path absolute_pcap = fs::absolute(pcap);
  const fs::path scenario_dir = absolute_pcap.parent_path();
  add_candidate(absolute_pcap);

  std::error_code ec;
  for (const auto & entry : fs::directory_iterator(scenario_dir, ec)) {
    add_candidate(entry.path());
  }

  const fs::path root_dir = scenario_dir.parent_path();
  ec.clear();
  for (const auto & scenario_entry : fs::directory_iterator(root_dir, ec)) {
    if (!scenario_entry.is_directory()) {
      continue;
    }

    add_candidate(scenario_entry.path() / absolute_pcap.filename());

    std::error_code child_ec;
    for (const auto & entry : fs::directory_iterator(scenario_entry.path(), child_ec)) {
      add_candidate(entry.path());
    }
  }

  return candidates;
}

RobosenseDecoderSeed preload_robosense_difop(const fs::path & pcap, const SensorSpec & spec)
{
  for (const auto & candidate : candidate_difop_pcaps(pcap, spec)) {
    if (auto seed = try_preload_robosense_difop(pcap, candidate, spec)) {
      return *seed;
    }
  }

  throw std::runtime_error(
    "No valid RoboSense DIFOP/info packet found in " + pcap.string() +
    " or same-sensor sibling PCAPs");
}

void convert_one(
  const fs::path & pcap, const fs::path & output, const SensorSpec & spec,
  const fs::path & calibration_root)
{
  const auto topic = pointcloud_topic(spec);
  std::cout << "CONVERT " << pcap << " -> " << output << " [" << sensor_name(spec)
            << " topic=" << topic << "]" << std::endl;

  const auto robosense_seed = spec.vendor == Vendor::Robosense
                                ? std::make_optional(preload_robosense_difop(pcap, spec))
                                : std::nullopt;
  McapPointcloudWriter writer(output, topic, spec.frame_id);
  ConversionStats stats;
  switch (spec.vendor) {
    case Vendor::Hesai:
      stats = decode_hesai(pcap, spec, calibration_root, writer);
      break;
    case Vendor::Robosense:
      stats = decode_robosense(pcap, spec, *robosense_seed, writer);
      break;
    case Vendor::Seyond:
      stats = decode_seyond(pcap, spec, calibration_root, writer);
      break;
  }

  std::cout << "DONE " << pcap << " udp=" << stats.udp_packets << " data=" << stats.data_packets
            << " info=" << stats.info_packets << " clouds=" << writer.clouds_written()
            << " points=" << writer.points_written() << std::endl;

  if (writer.clouds_written() == 0) {
    throw std::runtime_error("No pointclouds decoded from " + pcap.string());
  }
}

int run(int argc, char ** argv)
{
  try {
    const auto options = parse_args(argc, argv);
    if (!fs::is_regular_file(options.input_pcap)) {
      throw std::runtime_error("Input PCAP does not exist: " + options.input_pcap.string());
    }

    rclcpp::init(argc, argv);

    const auto specs = make_specs();
    const auto * spec = find_spec(options.input_pcap, specs);
    if (spec == nullptr) {
      throw std::runtime_error(
        "Unsupported PCAP filename prefix: " + options.input_pcap.filename().string());
    }

    const auto calibration_root =
      infer_calibration_root(options.input_pcap, *spec, options.calibration_root);
    convert_one(options.input_pcap, options.output_mcap, *spec, calibration_root);

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & ex) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "ERROR: " << ex.what() << std::endl;
    print_usage(argv[0]);
    return 2;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  return run(argc, argv);
}
