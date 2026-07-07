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
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <nebula_msgs/msg/nebula_packets.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <robosense_msgs/msg/robosense_info_packet.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
namespace fs = std::filesystem;
namespace drivers = nebula::drivers;

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

uint64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0) {
    return 0;
  }
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

template <typename MsgT>
MsgT deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  MsgT message;
  rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  rclcpp::Serialization<MsgT> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

class BagReader
{
public:
  explicit BagReader(const fs::path & input)
  : reader_(std::make_unique<rosbag2_cpp::readers::SequentialReader>())
  {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = input.string();
    storage_options.storage_id = "sqlite3";
    reader_.open(storage_options, {"cdr", "cdr"});
  }

  std::vector<rosbag2_storage::TopicMetadata> topics() const
  {
    return reader_.get_all_topics_and_types();
  }

  bool has_next() { return reader_.has_next(); }

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next()
  {
    try {
      return reader_.read_next();
    } catch (...) {
      return nullptr;
    }
  }

private:
  rosbag2_cpp::Reader reader_;
};

class McapPointcloudWriter
{
public:
  McapPointcloudWriter(const fs::path & output_path, bool force)
  {
    if (fs::exists(output_path)) {
      if (!force) {
        throw std::runtime_error(
          "Output already exists: " + output_path.string() + " (pass --force to overwrite)");
      }
      fs::remove_all(output_path);
    }
    if (!output_path.parent_path().empty()) {
      fs::create_directories(output_path.parent_path());
    }

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = output_path.string();
    storage_options.storage_id = "mcap";
    writer_.open(storage_options, {"cdr", "cdr"});
  }

  void create_topic(const std::string & topic)
  {
    writer_.create_topic({topic, "sensor_msgs/msg/PointCloud2", "cdr", ""});
  }

  size_t write(
    const drivers::NebulaPointCloudPtr & pointcloud, const std::string & topic,
    const std::string & frame_id, uint64_t timestamp_ns, uint64_t fallback_timestamp_ns)
  {
    if (!pointcloud || pointcloud->empty()) {
      return 0;
    }
    if (timestamp_ns == 0) {
      timestamp_ns = fallback_timestamp_ns;
    }

    auto msg = nebula::ros::to_ros_msg(*pointcloud);
    msg.header.frame_id = frame_id;
    msg.header.stamp = to_ros_stamp(timestamp_ns);
    writer_.write(msg, topic, to_rclcpp_time(timestamp_ns));
    ++clouds_written_;
    points_written_ += pointcloud->size();
    return pointcloud->size();
  }

  size_t clouds_written() const { return clouds_written_; }
  size_t points_written() const { return points_written_; }

private:
  rosbag2_cpp::Writer writer_;
  size_t clouds_written_{0};
  size_t points_written_{0};
};

enum class Vendor {
  Hesai,
  Robosense,
  Seyond,
};

struct SensorSpec
{
  std::string packet_topic;
  std::string info_topic;
  Vendor vendor;
  std::string model;
  std::string frame_id;
  fs::path calibration_relpath;
  double min_range{0.0};
  double max_range{300.0};
  uint16_t cloud_min_angle{0};
  uint16_t cloud_max_angle{360};
  uint16_t sync_angle{0};
  double cut_angle{360.0};
  uint16_t rotation_speed{600};
  std::string return_mode;
};

std::string infer_robosense_info_topic(const std::string & packet_topic)
{
  constexpr const char * packet_suffix = "/robosense_packets";
  constexpr const char * info_suffix = "/robosense_info_packets";
  if (
    packet_topic.size() >= std::char_traits<char>::length(packet_suffix) &&
    packet_topic.compare(
      packet_topic.size() - std::char_traits<char>::length(packet_suffix),
      std::char_traits<char>::length(packet_suffix), packet_suffix) == 0) {
    return packet_topic.substr(
             0, packet_topic.size() - std::char_traits<char>::length(packet_suffix)) +
           info_suffix;
  }
  return packet_topic + "_info";
}

std::vector<SensorSpec> make_specs()
{
  // OT128 and camera topics are intentionally omitted.
  // clang-format off
  return {
    {"/at128/pandar_packets", {}, Vendor::Hesai, "PandarAT128", "at128", "at128/PandarAT128.dat", 0.3, kUnfilteredMaxRangeMeters, 30, 150, 30, 150.0, 200, "Dual"},
    {"/ftx140/pandar_packets", {}, Vendor::Hesai, "FTX140", "ftx140", "hesai_ftx140/ftx140.dat", 0.05, kUnfilteredMaxRangeMeters, 20, 160, 20, 160.0, 600, "First"},
    {"/ftx180/pandar_packets", {}, Vendor::Hesai, "FTX180", "ftx180", "hesai_ftx180/ftx180.dat", 0.05, kUnfilteredMaxRangeMeters, 0, 180, 90, 180.0, 600, "First"},
    {"/e1/robosense_packets", "/e1/robosense_info_packets", Vendor::Robosense, "E1", "e1", {}, 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Strongest"},
    {"/hummingbirdd1/seyond_packets", {}, Vendor::Seyond, "HummingbirdD1", "hummingbirdd1", "seyond_hummingbirdd1/anglehv_table.bin", 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Single"},
    {"/robinw/seyond_packets", {}, Vendor::Seyond, "RobinW", "robinw", "seyond_robinw/anglehv_table.bin", 0.0, kUnfilteredMaxRangeMeters, 0, 360, 0, 360.0, 600, "Single"},
  };
  // clang-format on
}

std::string pointcloud_topic(const SensorSpec & spec)
{
  return "/" + spec.frame_id + "/pointcloud";
}

struct ConversionStats
{
  size_t input_messages{0};
  size_t packets{0};
  size_t info_packets{0};
  size_t clouds{0};
  size_t points{0};
};

std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> load_hesai_calibration(
  const SensorSpec & spec, const fs::path & calibration_root, drivers::SensorModel sensor_model)
{
  std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calibration;
  if (
    sensor_model == drivers::SensorModel::HESAI_FTX140 ||
    sensor_model == drivers::SensorModel::HESAI_FTX180) {
    calibration = std::make_shared<drivers::HesaiCorrectionFTX>();
  } else {
    calibration = std::make_shared<drivers::HesaiCorrection>();
  }

  const auto calibration_path = (calibration_root / spec.calibration_relpath).string();
  calibration->calibration_file = calibration_path;
  if (calibration->load_from_file(calibration_path) != nebula::Status::OK) {
    throw std::runtime_error("Failed to load Hesai calibration: " + calibration_path);
  }
  return calibration;
}

std::shared_ptr<const drivers::HesaiSensorConfiguration> make_hesai_config(
  const SensorSpec & spec, const fs::path & calibration_root)
{
  auto config = std::make_shared<drivers::HesaiSensorConfiguration>();
  config->sensor_model = drivers::sensor_model_from_string(spec.model);
  config->return_mode =
    drivers::return_mode_from_string_hesai(spec.return_mode, config->sensor_model);
  config->host_ip = "0.0.0.0";
  config->sensor_ip = "0.0.0.0";
  config->frame_id = spec.frame_id;
  config->packet_mtu_size = 1500;
  config->min_range = spec.min_range;
  config->max_range = spec.max_range;
  config->sync_angle = spec.sync_angle;
  config->cut_angle = spec.cut_angle;
  config->dual_return_distance_threshold = 0.1;
  config->calibration_path = (calibration_root / spec.calibration_relpath).string();
  config->calibration_download_enabled = false;
  config->rotation_speed = spec.rotation_speed;
  config->cloud_min_angle = spec.cloud_min_angle;
  config->cloud_max_angle = spec.cloud_max_angle;
  config->ptp_profile = drivers::PtpProfile::UNKNOWN_PROFILE;
  config->ptp_transport_type = drivers::PtpTransportType::UNKNOWN_TRANSPORT;
  config->ptp_switch_type = drivers::PtpSwitchType::UNKNOWN_SWITCH;
  config->hires_mode = false;
  return config;
}

std::shared_ptr<const drivers::RobosenseSensorConfiguration> make_robosense_config(
  const SensorSpec & spec)
{
  auto config = std::make_shared<drivers::RobosenseSensorConfiguration>();
  config->sensor_model = drivers::sensor_model_from_string(spec.model);
  config->return_mode = drivers::return_mode_from_string_robosense(spec.return_mode);
  config->host_ip = "0.0.0.0";
  config->sensor_ip = "0.0.0.0";
  config->data_port = 6699;
  config->gnss_port = 7788;
  config->difop2_port = 7788;
  config->frame_id = spec.frame_id;
  config->packet_mtu_size = 1500;
  config->min_range = spec.min_range;
  config->max_range = spec.max_range;
  config->scan_phase = 0.0;
  config->dual_return_distance_threshold = 0.1;
  return config;
}

bool directional_calibration_ready(
  drivers::SensorModel model, const drivers::RobosenseCalibrationConfiguration & calibration)
{
  if (model == drivers::SensorModel::ROBOSENSE_EMX) {
    return calibration.pixel_pitch.size() == 192 && calibration.surface_pitch_offset.size() == 2;
  }
  if (model == drivers::SensorModel::ROBOSENSE_EM4) {
    return calibration.pixel_pitch.size() == 520;
  }
  return true;
}

struct RobosenseSeed
{
  std::shared_ptr<const drivers::RobosenseSensorConfiguration> config;
  std::shared_ptr<const drivers::RobosenseCalibrationConfiguration> calibration;
  size_t info_packets{0};
};

RobosenseSeed preload_robosense_difop(const fs::path & input_bag, const SensorSpec & spec)
{
  auto base_config = make_robosense_config(spec);
  drivers::RobosenseInfoDriver info_driver(base_config);
  BagReader reader(input_bag);
  size_t info_packets = 0;

  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name != spec.info_topic) {
      continue;
    }

    const auto msg = deserialize<robosense_msgs::msg::RobosenseInfoPacket>(*bag_message);
    if (info_driver.decode_info_packet(msg.packet.data) != nebula::Status::OK) {
      continue;
    }
    ++info_packets;

    auto calibration = info_driver.get_sensor_calibration();
    if (!directional_calibration_ready(base_config->sensor_model, calibration)) {
      continue;
    }

    auto config = std::make_shared<drivers::RobosenseSensorConfiguration>(*base_config);
    const auto sensor_return_mode = info_driver.get_return_mode();
    if (sensor_return_mode != drivers::ReturnMode::UNKNOWN) {
      config->return_mode = sensor_return_mode;
    }
    config->use_sensor_time = info_driver.get_sync_status();
    calibration.create_corrected_channels();

    std::cout << "PRELOAD " << spec.frame_id << " decoded_info_packets=" << info_packets
              << std::endl;
    return {
      config,
      std::make_shared<const drivers::RobosenseCalibrationConfiguration>(std::move(calibration)),
      info_packets};
  }

  throw std::runtime_error("No valid RoboSense DIFOP/info packet found in " + input_bag.string());
}

drivers::SeyondSensorConfiguration make_seyond_config(const SensorSpec & spec)
{
  drivers::SeyondSensorConfiguration config;
  config.sensor_model = drivers::seyond_sensor_model_from_string(spec.model);
  if (config.sensor_model == drivers::SeyondSensorModel::UNKNOWN) {
    throw std::runtime_error("Unknown Seyond model: " + spec.model);
  }
  config.connection = {"0.0.0.0", "0.0.0.0", "", "", 0, 0, 0};
  config.use_sensor_time = true;
  config.frame_id = spec.frame_id;
  config.setup_sensor = false;
  config.return_mode = drivers::return_mode_from_string_seyond(spec.return_mode);
  return config;
}

struct Handler
{
  std::string input_topic;
  std::string output_topic;
  ConversionStats stats;
  std::function<void(const rosbag2_storage::SerializedBagMessage &)> process;
};

std::shared_ptr<Handler> make_hesai_handler(
  const SensorSpec & spec, const fs::path & calibration_root, McapPointcloudWriter & writer)
{
  const auto config = make_hesai_config(spec, calibration_root);
  const auto calibration = load_hesai_calibration(spec, calibration_root, config->sensor_model);
  auto handler = std::make_shared<Handler>();
  handler->input_topic = spec.packet_topic;
  handler->output_topic = pointcloud_topic(spec);

  auto logger = std::make_shared<drivers::loggers::RclcppLogger>("nebula_jari_tools.rosbag.hesai");
  auto last_packet_timestamp_ns = std::make_shared<uint64_t>(0);
  const auto frame_id = spec.frame_id;
  auto driver = std::make_shared<drivers::HesaiDriver>(
    config, calibration, logger,
    [&, handler, last_packet_timestamp_ns, frame_id](
      const drivers::NebulaPointCloudPtr & cloud, double ts) {
      const auto points = writer.write(
        cloud, handler->output_topic, frame_id, seconds_to_ns(ts), *last_packet_timestamp_ns);
      if (points != 0) {
        ++handler->stats.clouds;
        handler->stats.points += points;
      }
    });
  if (driver->get_status() != nebula::Status::OK) {
    throw std::runtime_error("Failed to initialize Hesai decoder for " + spec.model);
  }

  handler->process = [handler, driver, last_packet_timestamp_ns, spec,
                      &writer](const rosbag2_storage::SerializedBagMessage & bag_message) {
    ++handler->stats.input_messages;
    const auto scan = deserialize<pandar_msgs::msg::PandarScan>(bag_message);
    for (const auto & packet : scan.packets) {
      ++handler->stats.packets;
      *last_packet_timestamp_ns = stamp_to_ns(packet.stamp);
      const auto packet_size = std::min<size_t>(packet.size, packet.data.size());
      std::vector<uint8_t> data(packet.data.begin(), packet.data.begin() + packet_size);
      driver->parse_cloud_packet(data);
    }
  };
  return handler;
}

std::shared_ptr<Handler> make_robosense_handler(
  const fs::path & input_bag, const SensorSpec & spec, McapPointcloudWriter & writer)
{
  const auto seed = preload_robosense_difop(input_bag, spec);
  auto driver = std::make_shared<drivers::RobosenseDriver>(seed.config, seed.calibration);
  if (driver->get_status() != nebula::Status::OK) {
    throw std::runtime_error("Failed to initialize RoboSense decoder for " + spec.model);
  }

  auto handler = std::make_shared<Handler>();
  handler->input_topic = spec.packet_topic;
  handler->output_topic = pointcloud_topic(spec);
  handler->stats.info_packets = seed.info_packets;
  handler->process = [handler, driver, spec,
                      &writer](const rosbag2_storage::SerializedBagMessage & bag_message) {
    ++handler->stats.input_messages;
    const auto packets = deserialize<nebula_msgs::msg::NebulaPackets>(bag_message);
    for (const auto & packet : packets.packets) {
      ++handler->stats.packets;
      const auto pointcloud_ts = driver->parse_cloud_packet(packet.data);
      if (const auto pointcloud = std::get<0>(pointcloud_ts)) {
        const auto points = writer.write(
          pointcloud, handler->output_topic, spec.frame_id,
          seconds_to_ns(std::get<1>(pointcloud_ts)), stamp_to_ns(packet.stamp));
        if (points != 0) {
          ++handler->stats.clouds;
          handler->stats.points += points;
        }
      }
    }
  };
  return handler;
}

std::shared_ptr<Handler> make_seyond_handler(
  const SensorSpec & spec, const fs::path & calibration_root, McapPointcloudWriter & writer)
{
  auto handler = std::make_shared<Handler>();
  handler->input_topic = spec.packet_topic;
  handler->output_topic = pointcloud_topic(spec);

  auto last_packet_timestamp_ns = std::make_shared<uint64_t>(0);
  const auto calibration_path = calibration_root / spec.calibration_relpath;
  auto calibration_result =
    drivers::SeyondCalibrationData::load_from_file(calibration_path.string());
  if (!calibration_result.has_value()) {
    throw std::runtime_error("Failed to load Seyond calibration: " + calibration_path.string());
  }

  auto decoder = std::make_shared<drivers::SeyondDecoder>(
    make_seyond_config(spec),
    [&, handler, last_packet_timestamp_ns, frame_id = spec.frame_id](
      drivers::NebulaPointCloudPtr cloud, uint64_t base_ts) {
      const uint64_t cloud_ts =
        base_ts + (cloud && !cloud->empty() ? cloud->front().time_stamp : 0);
      const auto points =
        writer.write(cloud, handler->output_topic, frame_id, cloud_ts, *last_packet_timestamp_ns);
      if (points != 0) {
        ++handler->stats.clouds;
        handler->stats.points += points;
      }
    },
    calibration_result.value());

  handler->process = [handler, decoder, last_packet_timestamp_ns,
                      &writer](const rosbag2_storage::SerializedBagMessage & bag_message) {
    ++handler->stats.input_messages;
    const auto packets = deserialize<nebula_msgs::msg::NebulaPackets>(bag_message);
    for (const auto & packet : packets.packets) {
      ++handler->stats.packets;
      *last_packet_timestamp_ns = stamp_to_ns(packet.stamp);
      decoder->unpack(packet.data);
    }
  };
  return handler;
}

struct Options
{
  fs::path input_bag;
  fs::path output_mcap;
  std::optional<fs::path> calibration_root;
  std::vector<std::string> topics;
  std::optional<std::string> info_topic;
  bool force{false};
};

void print_usage(const char * argv0)
{
  std::cerr
    << "Usage: " << argv0
    << " INPUT_ROSBAG_DIR OUTPUT_MCAP [--topic PACKETS_TOPIC] [--info-topic INFO_TOPIC]\n"
    << "       " << argv0 << " INPUT_ROSBAG_DIR OUTPUT_MCAP --force [--calibration-root PATH]\n"
    << "  With multiple selected packet topics, OUTPUT_MCAP is used as a parent directory.\n";
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
    } else if (arg == "--topic") {
      options.topics.push_back(need_value(arg));
    } else if (arg == "--info-topic") {
      options.info_topic = need_value(arg);
    } else if (arg == "--force") {
      options.force = true;
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
    throw std::runtime_error("Expected INPUT_ROSBAG_DIR and OUTPUT_MCAP");
  }
  options.input_bag = positional[0];
  options.output_mcap = positional[1];
  if (options.info_topic && options.topics.size() != 1) {
    throw std::runtime_error("--info-topic requires exactly one --topic");
  }
  return options;
}

fs::path infer_calibration_root(
  const fs::path & input_bag, const std::optional<fs::path> & override_root)
{
  if (override_root) {
    return *override_root;
  }

  for (auto dir = fs::absolute(input_bag); !dir.empty(); dir = dir.parent_path()) {
    const auto candidate = dir / "calibration_files";
    if (fs::exists(candidate / "at128/PandarAT128.dat")) {
      return candidate;
    }
    if (dir == dir.root_path()) {
      break;
    }
  }
  throw std::runtime_error("Could not infer calibration root; pass --calibration-root PATH");
}

std::shared_ptr<Handler> make_handler(
  const fs::path & input_bag, const fs::path & calibration_root, const SensorSpec & spec,
  McapPointcloudWriter & writer)
{
  writer.create_topic(pointcloud_topic(spec));
  switch (spec.vendor) {
    case Vendor::Hesai:
      return make_hesai_handler(spec, calibration_root, writer);
    case Vendor::Robosense:
      return make_robosense_handler(input_bag, spec, writer);
    case Vendor::Seyond:
      return make_seyond_handler(spec, calibration_root, writer);
  }
  throw std::runtime_error("Unsupported sensor vendor");
}

std::vector<SensorSpec> select_specs(
  const Options & options, const std::vector<std::string> & bag_topics)
{
  const auto specs = make_specs();
  std::vector<SensorSpec> selected;
  const auto bag_has_topic = [&](const std::string & topic) {
    return std::find(bag_topics.begin(), bag_topics.end(), topic) != bag_topics.end();
  };

  if (options.topics.empty()) {
    for (auto spec : specs) {
      if (bag_has_topic(spec.packet_topic)) {
        if (spec.vendor == Vendor::Robosense && spec.info_topic.empty()) {
          spec.info_topic = infer_robosense_info_topic(spec.packet_topic);
        }
        selected.push_back(std::move(spec));
      }
    }
    return selected;
  }

  for (const auto & topic : options.topics) {
    const auto found = std::find_if(specs.begin(), specs.end(), [&](const SensorSpec & spec) {
      return spec.packet_topic == topic;
    });
    if (found == specs.end()) {
      throw std::runtime_error("Unsupported packet topic: " + topic);
    }
    if (!bag_has_topic(topic)) {
      throw std::runtime_error("Packet topic not found in input bag: " + topic);
    }

    SensorSpec spec = *found;
    if (spec.vendor == Vendor::Robosense) {
      spec.info_topic = options.info_topic.value_or(infer_robosense_info_topic(spec.packet_topic));
      if (!bag_has_topic(spec.info_topic)) {
        throw std::runtime_error("RoboSense info topic not found in input bag: " + spec.info_topic);
      }
    } else if (options.info_topic) {
      throw std::runtime_error("--info-topic is only valid for RoboSense packet topics");
    }
    selected.push_back(std::move(spec));
  }

  return selected;
}

struct ActiveConversion
{
  SensorSpec spec;
  fs::path output;
  std::shared_ptr<McapPointcloudWriter> writer;
  std::shared_ptr<Handler> handler;
};

fs::path output_for_spec(
  const fs::path & output_root, const SensorSpec & spec, size_t selected_count)
{
  if (selected_count == 1) {
    return output_root;
  }
  return output_root / spec.frame_id;
}

int run(int argc, char ** argv)
{
  try {
    const auto options = parse_args(argc, argv);
    if (!fs::is_directory(options.input_bag) || !fs::exists(options.input_bag / "metadata.yaml")) {
      throw std::runtime_error("Input rosbag directory is invalid: " + options.input_bag.string());
    }

    rclcpp::init(argc, argv);
    const auto calibration_root =
      infer_calibration_root(options.input_bag, options.calibration_root);

    BagReader metadata_reader(options.input_bag);
    std::vector<std::string> bag_topics;
    for (const auto & topic : metadata_reader.topics()) {
      bag_topics.push_back(topic.name);
    }
    const auto selected_specs = select_specs(options, bag_topics);
    if (selected_specs.empty()) {
      throw std::runtime_error("No supported packet topics found in input bag");
    }
    if (
      selected_specs.size() > 1 && fs::exists(options.output_mcap) &&
      !fs::is_directory(options.output_mcap)) {
      if (!options.force) {
        throw std::runtime_error(
          "Output path exists and is not a directory: " + options.output_mcap.string() +
          " (pass --force to overwrite)");
      }
      fs::remove_all(options.output_mcap);
    }

    std::vector<ActiveConversion> conversions;
    conversions.reserve(selected_specs.size());
    for (const auto & spec : selected_specs) {
      const auto output = output_for_spec(options.output_mcap, spec, selected_specs.size());
      auto writer = std::make_shared<McapPointcloudWriter>(output, options.force);
      auto handler = make_handler(options.input_bag, calibration_root, spec, *writer);
      conversions.push_back({spec, output, writer, handler});
      std::cout << "CONVERT " << spec.packet_topic << " -> " << output
                << " topic=" << handler->output_topic << std::endl;
    }

    std::unordered_map<std::string, std::shared_ptr<Handler>> by_topic;
    for (const auto & conversion : conversions) {
      by_topic.emplace(conversion.handler->input_topic, conversion.handler);
    }

    BagReader reader(options.input_bag);
    while (reader.has_next()) {
      const auto bag_message = reader.read_next();
      if (!bag_message) {
        break;
      }
      const auto found = by_topic.find(bag_message->topic_name);
      if (found == by_topic.end()) {
        continue;
      }
      found->second->process(*bag_message);
    }

    size_t total_clouds = 0;
    size_t total_points = 0;
    for (const auto & conversion : conversions) {
      const auto & handler = conversion.handler;
      total_clouds += conversion.writer->clouds_written();
      total_points += conversion.writer->points_written();
      std::cout << "DONE " << conversion.output << " input=" << handler->input_topic
                << " output=" << handler->output_topic << " messages=" << handler->stats.input_messages
                << " packets=" << handler->stats.packets << " info=" << handler->stats.info_packets
                << " clouds=" << handler->stats.clouds << " points=" << handler->stats.points
                << std::endl;
    }
    std::cout << "SUMMARY sensors=" << conversions.size() << " clouds=" << total_clouds
              << " points=" << total_points << std::endl;

    rclcpp::shutdown();
    return total_clouds == 0 ? 1 : 0;
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
