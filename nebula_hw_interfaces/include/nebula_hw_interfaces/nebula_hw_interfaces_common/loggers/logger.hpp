#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

class Logger
{
public:
  virtual void debug(const std::string & message);
  virtual void info(const std::string & message);
  virtual void warning(const std::string & message);
  virtual void error(const std::string & message);
};

}  // namespace drivers
}  // namespace nebula
