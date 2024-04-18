#include "nebula_ros/hesai/decoder_wrapper.hpp"

namespace nebula
{
namespace ros
{

HesaiDecoderWrapper::HesaiDecoderWrapper(rclcpp::Node* const parent_node,
                                         const std::shared_ptr<nebula::drivers::HesaiHwInterface>& hw_interface,
                                         std::shared_ptr<nebula::drivers::HesaiSensorConfiguration>& config)
  : status_(nebula::Status::NOT_INITIALIZED)
  , logger_(parent_node->get_logger().get_child("HesaiDecoder"))
  , hw_interface_(hw_interface)
  , sensor_cfg_(config)
{
  if (!config)
  {
    throw std::runtime_error("HesaiDecoderWrapper cannot be instantiated without a valid config!");
  }

  if (config->sensor_model == drivers::SensorModel::HESAI_PANDARAT128)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    parent_node->declare_parameter<std::string>("correction_file", "", descriptor);
    calibration_file_path_ = parent_node->get_parameter("correction_file").as_string();
  }
  else
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    parent_node->declare_parameter<std::string>("calibration_file", "", descriptor);
    calibration_file_path_ = parent_node->get_parameter("calibration_file").as_string();
  }

  auto calibration_result = GetCalibrationData();

  if (!calibration_result.has_value())
  {
    throw std::runtime_error(
        (std::stringstream() << "No valid calibration found: " << calibration_result.error()).str());
  }

  calibration_cfg_ptr_ = calibration_result.value();
  RCLCPP_INFO_STREAM(logger_, "Using calibration data from " << calibration_cfg_ptr_->calibration_file);

  RCLCPP_INFO(logger_, "Starting Decoder");

  driver_ptr_ = std::make_shared<drivers::HesaiDriver>(config, calibration_cfg_ptr_);
  status_ = driver_ptr_->GetStatus();

  if (Status::OK != status_)
  {
    throw std::runtime_error((std::stringstream() << "Error instantiating decoder: " << status_).str());
  }

  // Publish packets only if HW interface is connected
  if (hw_interface_)
  {
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
    packets_pub_ =
        parent_node->create_publisher<pandar_msgs::msg::PandarScan>("pandar_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", pointcloud_qos);
  aw_points_base_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", pointcloud_qos);
  aw_points_ex_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", pointcloud_qos);

  RCLCPP_INFO_STREAM(logger_, ". Wrapper=" << status_);
}

nebula::util::expected<std::shared_ptr<drivers::HesaiCalibrationConfigurationBase>, nebula::Status>
HesaiDecoderWrapper::GetCalibrationData()
{
  std::shared_ptr<drivers::HesaiCalibrationConfigurationBase> calib;

  if (sensor_cfg_->sensor_model == drivers::SensorModel::HESAI_PANDARAT128)
  {
    calib = std::make_shared<drivers::HesaiCorrection>();
  }
  else
  {
    calib = std::make_shared<drivers::HesaiCalibrationConfiguration>();
  }

  bool hw_connected = hw_interface_ != nullptr;
  std::string calibration_file_path_from_sensor;

  {
    int ext_pos = calibration_file_path_.find_last_of('.');
    calibration_file_path_from_sensor = calibration_file_path_.substr(0, ext_pos);
    // TODO: if multiple different sensors of the same type are used, this will mix up their calibration data
    calibration_file_path_from_sensor += "_from_sensor";
    calibration_file_path_from_sensor +=
        calibration_file_path_.substr(ext_pos, calibration_file_path_.size() - ext_pos);
  }

  // If a sensor is connected, try to download and save its calibration data
  if (hw_connected)
  {
    try
    {
      auto raw_data = hw_interface_->GetLidarCalibrationBytes();
      RCLCPP_INFO(logger_, "Downloaded calibration data from sensor.");
      auto status = calib->SaveToFileFromBytes(calibration_file_path_from_sensor, raw_data);
      if (status != Status::OK)
      {
        RCLCPP_ERROR_STREAM(logger_, "Could not save calibration data: " << status);
      }
      else
      {
        RCLCPP_INFO_STREAM(logger_, "Saved downloaded data to " << calibration_file_path_from_sensor);
      }
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Could not download calibration data: " << e.what());
    }
  }

  // If saved calibration data from a sensor exists (either just downloaded above, or previously), try to load it
  if (std::filesystem::exists(calibration_file_path_from_sensor))
  {
    auto status = calib->LoadFromFile(calibration_file_path_from_sensor);
    if (status == Status::OK)
    {
      calib->calibration_file = calibration_file_path_from_sensor;
      return calib;
    }

    RCLCPP_ERROR_STREAM(logger_, "Could not load downloaded calibration data: " << status);
  }
  else
  {
    RCLCPP_ERROR(logger_, "No downloaded calibration data found.");
  }

  RCLCPP_WARN(logger_, "Falling back to generic calibration file.");

  // If downloaded data did not exist or could not be loaded, fall back to a generic file.
  // If that file does not exist either, return an error code
  if (!std::filesystem::exists(calibration_file_path_))
  {
    RCLCPP_ERROR(logger_, "No calibration data found.");
    return nebula::Status(Status::INVALID_CALIBRATION_FILE);
  }

  // Try to load the existing fallback calibration file. Return an error if this fails
  auto status = calib->LoadFromFile(calibration_file_path_);
  if (status != Status::OK)
  {
    RCLCPP_ERROR(logger_, "Could not load fallback calibration file.");
    return status;
  }

  // Return the fallback calibration file
  calib->calibration_file = calibration_file_path_;
  return calib;
}

void HesaiDecoderWrapper::ProcessCloudPacket(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // Accumulate packets for recording only if someone is subscribed to the topic (for performance)
  if (hw_interface_ &&
      (packets_pub_->get_subscription_count() > 0 || packets_pub_->get_intra_process_subscription_count() > 0))
  {
    if (current_scan_msg_->packets.size() == 0)
    {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }

    pandar_msgs::msg::PandarPacket pandar_packet_msg{};
    pandar_packet_msg.stamp = packet_msg->stamp;
    pandar_packet_msg.size = packet_msg->data.size();
    std::copy(packet_msg->data.begin(), packet_msg->data.end(), pandar_packet_msg.data.begin());
    current_scan_msg_->packets.emplace_back(std::move(pandar_packet_msg));
  }

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts =
      driver_ptr_->ParseCloudPacket(packet_msg->data);
  nebula::drivers::NebulaPointCloudPtr pointcloud = std::get<0>(pointcloud_ts);

  if (pointcloud == nullptr)
  {
    // todo
    // RCLCPP_WARN_STREAM(logger_, "Empty cloud parsed.");
    return;
  };

  // Publish scan message only if it has been written to
  if (current_scan_msg_ && !current_scan_msg_->packets.empty())
  {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<pandar_msgs::msg::PandarScan>();
  }

  if (nebula_points_pub_->get_subscription_count() > 0 ||
      nebula_points_pub_->get_intra_process_subscription_count() > 0)
  {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (aw_points_base_pub_->get_subscription_count() > 0 ||
      aw_points_base_pub_->get_intra_process_subscription_count() > 0)
  {
    const auto autoware_cloud_xyzi = nebula::drivers::convertPointXYZIRCAEDTToPointXYZIR(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (aw_points_ex_pub_->get_subscription_count() > 0 || aw_points_ex_pub_->get_intra_process_subscription_count() > 0)
  {
    const auto autoware_ex_cloud =
        nebula::drivers::convertPointXYZIRCAEDTToPointXYZIRADT(pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp = rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }
}

void HesaiDecoderWrapper::PublishCloud(std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
                                       const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
  if (pointcloud->header.stamp.sec < 0)
  {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

nebula::Status HesaiDecoderWrapper::Status()
{
  if (!driver_ptr_)
  {
    return nebula::Status::NOT_INITIALIZED;
  }

  return driver_ptr_->GetStatus();
}
}  // namespace ros
}  // namespace nebula