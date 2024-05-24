#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/http.hpp"

namespace nebula
{
namespace drivers
{

HTTPConnection::HTTPConnection(const std::string & sensor_ip, uint16_t sensor_port)
: driver_(std::make_shared<boost::asio::io_context>(1))
{
  driver_.init_client(sensor_ip, sensor_port);
}

HTTPConnection::http_response_t HTTPConnection::get(const std::string & endpoint)
{
  return driver_.get(endpoint);
}

HTTPConnection::http_response_t HTTPConnection::post(
  const std::string & endpoint, const std::string & body)
{
  return driver_.post(endpoint, body);
}

}  // namespace drivers
}  // namespace nebula
