#pragma once

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <iostream>

#include <chrono>

namespace nebula
{
namespace ros
{
class TopicDelay final : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

public:
  TopicDelay(const rclcpp::NodeOptions & options);

  void ReceiveScanMsgCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan_msg);
};

}  // namespace ros
}  // namespace nebula

