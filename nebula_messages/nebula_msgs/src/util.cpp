#include "nebula_msgs/nebula_msgs_util/util.hpp"

template <typename ClockT>
builtin_interfaces::msg::Time nebula_msgs::util::make_timestamp(
  const std::chrono::time_point<ClockT> & time_point)
{
  auto time_since_epoch = time_point.time_since_epoch();
  auto stamp_s = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch).count();
  auto stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch).count();

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_s);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000);
  return stamp;
}
