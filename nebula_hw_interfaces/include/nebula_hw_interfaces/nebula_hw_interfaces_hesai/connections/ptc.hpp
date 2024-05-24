#pragma once

#include <boost_tcp_driver/tcp_driver.hpp>
#include <nebula_common/util/expected.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <mutex>

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/loggers/logger.hpp"

namespace nebula
{
namespace drivers
{

class PTCConnection
{
  static const uint8_t PTC_COMMAND_DUMMY_BYTE = 0x00;
  static const uint8_t PTC_COMMAND_HEADER_HIGH = 0x47;
  static const uint8_t PTC_COMMAND_HEADER_LOW = 0x74;

  static const uint8_t PTC_ERROR_CODE_NO_ERROR = 0x00;
  static const uint8_t PTC_ERROR_CODE_INVALID_INPUT_PARAM = 0x01;
  static const uint8_t PTC_ERROR_CODE_SERVER_CONN_FAILED = 0x02;
  static const uint8_t PTC_ERROR_CODE_INVALID_DATA = 0x03;
  static const uint8_t PTC_ERROR_CODE_OUT_OF_MEMORY = 0x04;
  static const uint8_t PTC_ERROR_CODE_UNSUPPORTED_CMD = 0x05;
  static const uint8_t PTC_ERROR_CODE_FPGA_COMM_FAILED = 0x06;
  static const uint8_t PTC_ERROR_CODE_OTHER = 0x07;

  static const uint8_t TCP_ERROR_UNRELATED_RESPONSE = 1;
  static const uint8_t TCP_ERROR_UNEXPECTED_PAYLOAD = 2;
  static const uint8_t TCP_ERROR_TIMEOUT = 4;
  static const uint8_t TCP_ERROR_INCOMPLETE_RESPONSE = 8;
public:

  struct ptc_error_t
  {
    uint8_t error_flags = 0;
    uint8_t ptc_error_code = 0;

    bool ok() { return !error_flags && !ptc_error_code; }
  };

  using ptc_cmd_result_t = nebula::util::expected<std::vector<uint8_t>, ptc_error_t> ;

  PTCConnection(const std::string & sensor_ip, uint16_t sensor_port, Logger logger);

  ptc_cmd_result_t executeCommand(uint8_t command_id, const std::vector<uint8_t> & payload);

private:
  ::drivers::tcp_driver::TcpDriver driver_;
  std::mutex mtx_inflight_tcp_request_;
  Logger logger_;
};

}  // namespace drivers
}  // namespace nebula
