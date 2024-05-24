#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"

namespace nebula
{
namespace drivers
{

UDPConnection::UDPConnection(
  std::string sensor_ip, std::string host_ip, uint16_t host_port,
  std::function<void(std::vector<uint8_t> &)> receive_callback) :
  driver_(::drivers::common::IoContext(1)),
  receive_callback_(receive_callback)
{
  driver_.init_receiver(host_ip, host_port);
  auto & receiver = *driver_.receiver();
  receiver.bind();
  receiver.open();
  receiver.asyncReceive(receive_callback);
}

void UDPConnection::send(std::vector<uint8_t> & data)
{
  driver_.sender()->send(data);
}

}  // namespace drivers
}  // namespace nebula
