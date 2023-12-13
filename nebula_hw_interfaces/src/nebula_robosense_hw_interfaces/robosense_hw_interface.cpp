#include "nebula_hw_interfaces/nebula_hw_interfaces_robosense/robosense_hw_interface.hpp"
namespace nebula
{
namespace drivers
{
RobosenseHwInterface::RobosenseHwInterface()
: cloud_io_context_{new ::drivers::common::IoContext(1)},
  info_io_context_{new ::drivers::common::IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  info_udp_driver_{new ::drivers::udp_driver::UdpDriver(*info_io_context_)},
  scan_cloud_ptr_{std::make_unique<nebula_msgs::msg::RawPacketArray>()}
{
}

void RobosenseHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  if (!scan_reception_callback_) {
    return;
  }

  if (!is_valid_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }

  auto now = std::chrono::system_clock::now();
  auto packet_stamped = scan_cloud_ptr_->packets.emplace_back();
  packet_stamped.stamp = nebula_msgs::util::make_timestamp(now);
  packet_stamped.packet.data = buffer;  // TODO(mojomex): maybe swap instead of copy?

  scan_reception_callback_(std::move(scan_cloud_ptr_));
  scan_cloud_ptr_ = std::make_unique<nebula_msgs::msg::RawPacketArray>();
}

void RobosenseHwInterface::ReceiveInfoPacketCallback(const std::vector<uint8_t> & buffer)
{
  if (!info_reception_callback_) {
    return;
  }

  if (!is_valid_info_packet_(buffer.size())) {
    PrintDebug("Invalid Packet: " + std::to_string(buffer.size()));
    return;
  }

  std::unique_ptr<nebula_msgs::msg::RawPacketStamped> packet =
    std::make_unique<nebula_msgs::msg::RawPacketStamped>();
  auto now = std::chrono::system_clock::now();

  packet->stamp = nebula_msgs::util::make_timestamp(now);
  packet->packet.data = buffer;  // TODO(mojomex): maybe swap instead of copy?

  info_reception_callback_(std::move(packet));
}

Status RobosenseHwInterface::CloudInterfaceStart()
{
  try {
    std::cout << "Starting UDP server for data packets on: " << *sensor_configuration_ << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();

    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&RobosenseHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status RobosenseHwInterface::InfoInterfaceStart()
{
  try {
    std::cout << "Starting UDP server for info packets on: " << *sensor_configuration_ << std::endl;
    PrintInfo(
      "Starting UDP server for info packets on: " + sensor_configuration_->sensor_ip + ":" +
      std::to_string(sensor_configuration_->gnss_port));
    info_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->gnss_port);
    info_udp_driver_->receiver()->open();
    info_udp_driver_->receiver()->bind();

    info_udp_driver_->receiver()->asyncReceive(
      std::bind(&RobosenseHwInterface::ReceiveInfoPacketCallback, this, std::placeholders::_1));

  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->gnss_port << std::endl;
    return status;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  return Status::OK;
}

Status RobosenseHwInterface::CloudInterfaceStop()
{
  return Status::ERROR_1;
}

Status RobosenseHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  Status status = Status::OK;

  try {
    sensor_configuration_ =
      std::static_pointer_cast<RobosenseSensorConfiguration>(sensor_configuration);

    if (
      sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_BPEARL ||
      sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_BPEARL_V3 ||
      sensor_configuration_->sensor_model == SensorModel::ROBOSENSE_BPEARL_V4) {
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == BPEARL_PACKET_SIZE); };
      is_valid_info_packet_ = [](size_t packet_size) {
        return (packet_size == BPEARL_INFO_PACKET_SIZE);
      };
    } else if (sensor_configuration->sensor_model == SensorModel::ROBOSENSE_HELIOS) {
      is_valid_packet_ = [](size_t packet_size) { return (packet_size == HELIOS_PACKET_SIZE); };
      is_valid_info_packet_ = [](size_t packet_size) {
        return (packet_size == HELIOS_INFO_PACKET_SIZE);
      };
    } else {
      status = Status::INVALID_SENSOR_MODEL;
    }
  } catch (const std::exception & ex) {
    status = Status::SENSOR_CONFIG_ERROR;
    std::cerr << status << std::endl;
    return status;
  }
  return status;
}

Status RobosenseHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status RobosenseHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  PrintDebug(calibration_configuration.calibration_file);
  return Status::ERROR_1;
}

Status RobosenseHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::RawPacketArray>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

Status RobosenseHwInterface::RegisterInfoCallback(
  std::function<void(std::unique_ptr<nebula_msgs::msg::RawPacketStamped>)> info_callback)
{
  info_reception_callback_ = std::move(info_callback);
  return Status::OK;
}

void RobosenseHwInterface::PrintDebug(std::string debug)
{
  if (parent_node_logger_) {
    RCLCPP_DEBUG_STREAM((*parent_node_logger_), debug);
  } else {
    std::cout << debug << std::endl;
  }
}

void RobosenseHwInterface::PrintInfo(std::string info)
{
  if (parent_node_logger_) {
    RCLCPP_INFO_STREAM((*parent_node_logger_), info);
  } else {
    std::cout << info << std::endl;
  }
}

void RobosenseHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger_ = logger;
}

}  // namespace drivers
}  // namespace nebula
