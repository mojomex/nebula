#pragma once

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/loggers/logger.hpp"

#include <rclcpp/logging.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

class RclCppLogger : public Logger
{
public:
  explicit RclCppLogger(const std::string & name) : logger_(rclcpp::get_logger(name)) {}
  explicit RclCppLogger(const rclcpp::Logger & logger) : logger_(logger) {}

  void debug(const std::string & message) override {
    RCLCPP_DEBUG(logger_, message.c_str());
  }
  void info(const std::string & message) override {
    RCLCPP_INFO(logger_, message.c_str());
  }
  void warning(const std::string & message) override {
    RCLCPP_WARN(logger_, message.c_str());
  }
  void error(const std::string & message) override {
    RCLCPP_ERROR(logger_, message.c_str());
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace drivers
}  // namespace nebula
