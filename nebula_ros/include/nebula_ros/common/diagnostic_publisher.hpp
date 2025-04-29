// Copyright 2024 TIER IV, Inc.
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

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <stdexcept>
#include <string>
#include <utility>

namespace nebula::ros
{

class DiagnosticPublisher
{
  DiagnosticPublisher(
    const rclcpp::Node::SharedPtr & node, std::string hardware_id, std::string frame_id)
  : publisher_(node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1)),
    node_(node),
    hardware_id_(std::move(hardware_id)),
    frame_id_(std::move(frame_id))
  {
    if (hardware_id_.empty()) throw std::invalid_argument("Non-empty hardware ID required");
    if (frame_id_.empty()) throw std::invalid_argument("Non-empty frame ID required");
  }

  /**
   * @brief Ensures correct hardware and frame ID and timestamp of `msg`, then publishes it.
   *
   * Hardware and frame ID are overwritten by the values this publisher has been initialized
   * with. The names of all `DiagnosticStatus`es are prefixed with the node's fully qualified
   * name, e.g. `my_status` --> `/my/node/name: my_status`.
   * If the header timestamp of `msg` is all zero, the current time is used. Otherwise, the
   * timestamp is preserved.
   *
   * @param msg The partially filled diagnostic array.
   */
  void publish(diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    if (msg.header.stamp == builtin_interfaces::msg::Time{}) {
      msg.header.stamp = node_->get_clock()->now();
    }

    msg.header.frame_id = frame_id_;
    for (auto & status : msg.status) {
      status.hardware_id = hardware_id_;
      status.name = node_->get_fully_qualified_name() + std::string(": ") + status.name;
    }

    publisher_->publish(msg);
  }

private:
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;
  std::string hardware_id_;
  std::string frame_id_;
};

}  // namespace nebula::ros
