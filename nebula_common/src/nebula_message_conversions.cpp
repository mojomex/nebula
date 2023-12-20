#include "nebula_common/nebula_message_conversions.hpp"

template <>
nebula_msgs::msg::RawPacketArray nebula::legacy_support::legacy_to_nebula_msg<>(
  const pandar_msgs::msg::PandarScan & scan_msg)
{
  nebula_msgs::msg::RawPacketArray nebula_msg;
  nebula_msg.packets.reserve(scan_msg.packets.size());
  for (const auto & scan_packet : scan_msg.packets) {
    auto & nebula_packet = nebula_msg.packets.emplace_back();
    nebula_packet.stamp = scan_packet.stamp;
    nebula_packet.packet.data.reserve(scan_packet.size);
    std::copy_n(
      scan_packet.data.begin(), scan_packet.size, std::back_inserter(nebula_packet.packet.data));
  }
  return nebula_msg;
}

template <>
nebula_msgs::msg::RawPacketArray nebula::legacy_support::legacy_to_nebula_msg<>(
  const velodyne_msgs::msg::VelodyneScan & scan_msg)
{
  nebula_msgs::msg::RawPacketArray nebula_msg;
  nebula_msg.packets.reserve(scan_msg.packets.size());
  for (const auto & scan_packet : scan_msg.packets) {
    auto & nebula_packet = nebula_msg.packets.emplace_back();
    nebula_packet.stamp = scan_packet.stamp;
    nebula_packet.packet.data.reserve(scan_packet.data.size());
    std::copy(
      scan_packet.data.begin(), scan_packet.data.end(),
      std::back_inserter(nebula_packet.packet.data));
  }
  return nebula_msg;
}
