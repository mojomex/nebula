#pragma once

#include <nebula_msgs/msg/raw_packet_array.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

namespace nebula
{
namespace legacy_support
{

/// @brief Convert a legacy scan message of type T into an equivalent RawPacketArray message.
template <typename T>
nebula_msgs::msg::RawPacketArray legacy_to_nebula_msg(const T & scan_msg);

}  // namespace legacy_support
}  // namespace nebula
