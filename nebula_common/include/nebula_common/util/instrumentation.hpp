#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <unordered_map>

namespace nebula::ros
{

template <
  class OutputUnit = std::chrono::seconds, class InternalUnit = std::chrono::microseconds,
  class Clock = std::chrono::steady_clock>
class StopWatch
{
public:
  StopWatch() { tic(default_name); }

  void tic(const std::string & name = default_name) { t_start_[name] = Clock::now(); }

  void tic(const char * name) { tic(std::string(name)); }

  double toc(const std::string & name, const bool reset = false)
  {
    const auto t_start = t_start_.at(name);
    const auto t_end = Clock::now();
    const auto duration = std::chrono::duration_cast<InternalUnit>(t_end - t_start).count();

    if (reset) {
      t_start_[name] = Clock::now();
    }

    const auto one_sec = std::chrono::duration_cast<InternalUnit>(OutputUnit(1)).count();

    return static_cast<double>(duration) / one_sec;
  }

  double toc(const char * name, const bool reset = false) { return toc(std::string(name), reset); }

  double toc(const bool reset = false) { return toc(default_name, reset); }

private:
  using Time = std::chrono::time_point<Clock>;
  static constexpr const char * default_name{"__auto__"};

  std::unordered_map<std::string, Time> t_start_;
};

namespace debug_publisher
{

template <class T_msg, class T>
T_msg toDebugMsg(const T & data, const rclcpp::Time & stamp)
{
  T_msg msg;
  msg.stamp = stamp;
  msg.data = data;
  return msg;
}
}  // namespace debug_publisher

class DebugPublisher
{
public:
  explicit DebugPublisher(rclcpp::Node * node, const char * ns) : node_(node), ns_(ns) {}

  template <class T>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    if (pub_map_.count(name) == 0) {
      pub_map_[name] = node_->create_publisher<T>(std::string(ns_) + "/" + name, qos);
    }

    std::dynamic_pointer_cast<rclcpp::Publisher<T>>(pub_map_.at(name))->publish(data);
  }

  template <class T_msg, class T>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    publish(name, debug_publisher::toDebugMsg<T_msg>(data, node_->now()), qos);
  }

private:
  rclcpp::Node * node_;
  const char * ns_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> pub_map_;
};

}  // namespace nebula::ros