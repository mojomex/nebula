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

#include <nebula_common/point_types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nebula::drivers
{

/**
 * @brief Owns a PCLPointCloud2 and ensures that the buffer is correctly filled, regardless of
 * whether the cloud is organized (height > 1) or not.
 */
class ManagedPointCloud
{
  struct OrganizedCloudMetadata
  {
    /// The end index (= the next free index) of each row in the pointcloud
    std::vector<size_t> row_ends;
  };

public:
  ManagedPointCloud() : cloud_(get_empty_cloud())
  {
    cloud_.point_step = sizeof(NebulaPoint);
    cloud_.height = 1;
    cloud_.width = cloud_.row_step = 0;
  }

  explicit ManagedPointCloud(size_t width, size_t height) : cloud_(get_empty_cloud())
  {
    cloud_.point_step = sizeof(NebulaPoint);
    cloud_.width = width;
    cloud_.height = height;
    cloud_.row_step = width * cloud_.point_step;
    cloud_.data.resize(cloud_.point_step * cloud_.width * cloud_.height);
    organized_metadata_.emplace(OrganizedCloudMetadata());
    organized_metadata_->row_ends.resize(height, 0);
  }

  void push_back(const NebulaPoint & p)
  {
    if (!organized_metadata_.has_value()) {
      size_t index = cloud_.data.size() / sizeof(p);
      size_t new_size = cloud_.data.size() + sizeof(p);
      cloud_.data.resize(new_size);
      cloud_.width += 1;
      cloud_.row_step = cloud_.width * cloud_.point_step;
      (*this)[index] = p;
      return;
    }

    uint16_t channel = p.channel;
    auto & row_end = organized_metadata_->row_ends.at(channel);
    size_t index = channel * cloud_.width + row_end;
    (*this)[index] = p;
    row_end++;
  }

  /**
   * @brief For unorganized pointclouds, clears all points and sets the width and row step to 0. For
   * organized pointclouds, overwrites all points with 0 but leaves the dimensions intact.
   */
  void reset()
  {
    if (!organized_metadata_.has_value()) {
      cloud_.width = cloud_.row_step = 0;
      cloud_.data.clear();
      return;
    }

    size_t expected_size = cloud_.point_step * cloud_.width * cloud_.height;
    if (cloud_.data.size() != expected_size) {
      // If the data was swapped with some other buffer, erase fully and resize
      cloud_.data.clear();
      cloud_.data.resize(expected_size);
    } else {
      // If the buffer is still the right size, erase only the areas that contained valid points
      for (size_t row = 0; row < cloud_.height; ++row) {
        size_t row_start_index = row * cloud_.width;
        size_t row_extent = organized_metadata_->row_ends.at(row);
        std::fill_n(&cloud_.data[row_start_index], row_extent, 0);
      }
    }

    std::fill(organized_metadata_->row_ends.begin(), organized_metadata_->row_ends.end(), 0);
  }

  NebulaPoint & at(size_t index)
  {
    if (cloud_.data.size() < (index + 1) * sizeof(NebulaPoint)) {
      throw std::out_of_range("Accessed pointcloud at an index past its end");
    }

    return (*this)[index];
  }

  NebulaPoint & at(size_t row, size_t column)
  {
    if (!organized_metadata_.has_value()) {
      throw std::runtime_error("Tried accessing an unorganized pointcloud with row/column indices");
    }

    size_t row_end = organized_metadata_->row_ends.at(row);
    if (column >= row_end) {
      throw std::out_of_range("Tried to access a row at a column past its end");
    }

    size_t index = row * cloud_.width + column;
    return (*this)[index];
  }

  /**
   * @brief Returns the pointcloud and resets the internal buffer. Points are moved, not copied.
   *
   * @return pcl::PCLPointCloud2 The popped pointcloud
   */
  sensor_msgs::msg::PointCloud2 pop_cloud()
  {
    sensor_msgs::msg::PointCloud2 result;
    std::swap(result, cloud_);
    cloud_.data.clear();
    reset();
    return result;
  }

  /**
   * @brief Returns a constant reference to the current pointcloud.
   *
   * @return const pcl::PCLPointCloud2& The current pointcloud
   */
  const sensor_msgs::msg::PointCloud2 & peek_cloud() { return cloud_; }

private:
  NebulaPoint & operator[](size_t index)
  {
    return *reinterpret_cast<NebulaPoint *>(&cloud_.data[index * sizeof(NebulaPoint)]);
  }

  static sensor_msgs::msg::PointCloud2 get_empty_cloud()
  {
    NebulaPointCloud cloud_template;
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(cloud_template, cloud);
    return cloud;
  }

  sensor_msgs::msg::PointCloud2 cloud_;
  std::optional<OrganizedCloudMetadata> organized_metadata_;
};

}  // namespace nebula::drivers
