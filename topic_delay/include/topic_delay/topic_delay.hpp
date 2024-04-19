#pragma once

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pandar_msgs/msg/pandar_scan.hpp"
#include <iostream>

#include <chrono>

namespace nebula
{
namespace ros
{
class TopicDelay final : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<pandar_msgs::msg::PandarScan>::SharedPtr scan_sub_;

public:
  TopicDelay(const rclcpp::NodeOptions & options);

  void ReceiveCloudMsgCallback(const sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg);

  void ReceiveScanMsgCallback(const pandar_msgs::msg::PandarScan::UniquePtr scan_msg);
};

}  // namespace ros
}  // namespace nebula

