#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>

namespace nebula_msgs
{
namespace util
{
template <typename ClockT>
builtin_interfaces::msg::Time make_timestamp(const std::chrono::time_point<ClockT> & time_point);
}  // namespace util
}  // namespace nebula_msgs
