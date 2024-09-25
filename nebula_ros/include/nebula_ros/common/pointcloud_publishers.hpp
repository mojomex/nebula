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

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <utility>

namespace nebula::ros
{

class PointCloudPublisher
{
public:
  virtual void publish(sensor_msgs::msg::PointCloud2::UniquePtr cloud) = 0;
  virtual void publish_if_subscribers_exist(sensor_msgs::msg::PointCloud2::UniquePtr cloud) = 0;
};

/**
 * @briefPublishes PointCloud2 messages via ROS 2 normally.
 */
class BasicPointCloudPublisher : public PointCloudPublisher
{
  using publisher_ptr_t = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

public:
  explicit BasicPointCloudPublisher(publisher_ptr_t publisher) : pub_(std::move(publisher)) {}

  void publish(sensor_msgs::msg::PointCloud2::UniquePtr cloud) override { pub_->publish(*cloud); }

  void publish_if_subscribers_exist(sensor_msgs::msg::PointCloud2::UniquePtr cloud) override
  {
    if (pub_->get_subscription_count() == 0 && pub_->get_intra_process_subscription_count() == 0) {
      return;
    }

    publish(std::move(cloud));
  }

private:
  publisher_ptr_t pub_;
};

/**
 * @brief Converts and publishes PointCloud2 messages to CUDA.
 */
class CudaPointCloudPublisher : public PointCloudPublisher
{
  using publisher_ptr_t =
    std::shared_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>;

public:
  explicit CudaPointCloudPublisher(publisher_ptr_t publisher) : pub_(std::move(publisher)) {}

  void publish(sensor_msgs::msg::PointCloud2::UniquePtr cloud) override
  {
    auto cuda_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>(*cloud);
    pub_->publish(std::move(cuda_cloud));
  }

  void publish_if_subscribers_exist(sensor_msgs::msg::PointCloud2::UniquePtr cloud) override
  {
    if (pub_->get_subscription_count() == 0 && pub_->get_intra_process_subscription_count() == 0) {
      return;
    }

    publish(std::move(cloud));
  }

private:
  publisher_ptr_t pub_;
};

}  // namespace nebula::ros
