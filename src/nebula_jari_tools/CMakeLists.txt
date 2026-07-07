cmake_minimum_required(VERSION 3.20)
project(nebula_jari_tools)

find_package(ament_cmake REQUIRED)
find_package(nebula_core_common REQUIRED)
find_package(nebula_core_decoders REQUIRED)
find_package(nebula_core_ros REQUIRED)
find_package(nebula_hesai_common REQUIRED)
find_package(nebula_hesai_decoders REQUIRED)
find_package(nebula_msgs REQUIRED)
find_package(nebula_robosense_common REQUIRED)
find_package(nebula_robosense_decoders REQUIRED)
find_package(nebula_seyond_common REQUIRED)
find_package(nebula_seyond_decoders REQUIRED)
find_package(pandar_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robosense_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(jari_pcap_to_mcap src/jari_pcap_to_mcap.cpp)
target_compile_features(jari_pcap_to_mcap PRIVATE cxx_std_17)
target_link_libraries(
  jari_pcap_to_mcap
  PRIVATE
    nebula_core_common::nebula_core_common
    nebula_core_decoders::nebula_core_decoders
    nebula_core_ros::nebula_core_ros
    nebula_hesai_common::nebula_hesai_common
    nebula_hesai_decoders::nebula_hesai_decoders
    nebula_robosense_common::nebula_robosense_common
    nebula_robosense_decoders::nebula_robosense_decoders
    nebula_robosense_decoders::nebula_robosense_decoders_info
    nebula_seyond_common::nebula_seyond_common
    nebula_seyond_decoders::nebula_seyond_decoders
    rclcpp::rclcpp
    rosbag2_cpp::rosbag2_cpp)
ament_target_dependencies(jari_pcap_to_mcap PUBLIC sensor_msgs)

add_executable(jari_rosbag_to_mcap src/jari_rosbag_to_mcap.cpp)
target_compile_features(jari_rosbag_to_mcap PRIVATE cxx_std_17)
target_link_libraries(
  jari_rosbag_to_mcap
  PRIVATE
    nebula_core_common::nebula_core_common
    nebula_core_decoders::nebula_core_decoders
    nebula_core_ros::nebula_core_ros
    nebula_hesai_common::nebula_hesai_common
    nebula_hesai_decoders::nebula_hesai_decoders
    nebula_robosense_common::nebula_robosense_common
    nebula_robosense_decoders::nebula_robosense_decoders
    nebula_robosense_decoders::nebula_robosense_decoders_info
    nebula_seyond_common::nebula_seyond_common
    nebula_seyond_decoders::nebula_seyond_decoders
    rclcpp::rclcpp
    rosbag2_cpp::rosbag2_cpp)
ament_target_dependencies(
  jari_rosbag_to_mcap
  PUBLIC
    nebula_msgs
    pandar_msgs
    robosense_msgs
    sensor_msgs)

install(TARGETS jari_pcap_to_mcap jari_rosbag_to_mcap DESTINATION lib/${PROJECT_NAME})

ament_package()
