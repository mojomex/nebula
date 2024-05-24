#pragma once

#include <boost_udp_driver/udp_driver.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

class UDPConnection
{
public:
  UDPConnection(
    std::string sensor_ip, std::string host_ip, uint16_t host_port,
    std::function<void(std::vector<uint8_t> &)> receive_callback);

  void send(std::vector<uint8_t> & data);

private:
  ::drivers::udp_driver::UdpDriver driver_;
  std::function<void(std::vector<uint8_t> &)> receive_callback_;
};

}  // namespace drivers
}  // namespace nebula
