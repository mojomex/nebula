#pragma once

#include <boost_tcp_driver/http_client_driver.hpp>
#include <nebula_common/util/expected.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

class HTTPConnection
{
public:
  using http_response_t = nebula::util::expected<std::string, int>;

  HTTPConnection(const std::string & sensor_ip, uint16_t sensor_port);

  http_response_t get(const std::string & endpoint);

  http_response_t post(const std::string & endpoint, const std::string & body);

private:
  ::drivers::tcp_driver::HttpClientDriver driver_;
};

}  // namespace drivers
}  // namespace nebula
