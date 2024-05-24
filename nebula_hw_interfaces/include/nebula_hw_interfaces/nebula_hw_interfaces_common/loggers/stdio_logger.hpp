#pragma once

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/loggers/logger.hpp"

#include <cstdint>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

class StdIoLogger : public Logger
{
public:
  explicit StdIoLogger(const std::string & tag) : tag_(tag) {}

  void debug(const std::string & message) override { log(std::cout, "DEBUG", message); }
  void info(const std::string & message) override { log(std::cout, "INFO", message); }
  void warning(const std::string & message) override { log(std::cerr, "WARN", message); }
  void error(const std::string & message) override { log(std::cerr, "ERROR", message); }

private:
  void log(std::ostream & os, const std::string & severity, const std::string & message)
  {
    os << '[' << severity << "][" << tag_ << ']' << message << std::endl;
  }

  const std::string tag_;
};

}  // namespace drivers
}  // namespace nebula
